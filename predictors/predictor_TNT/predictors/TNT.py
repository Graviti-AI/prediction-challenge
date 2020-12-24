import torch
from torch import nn
import torch.nn.functional as F
from predictors.vector_net import VectorNet
from predictors.sub_graph import SubGraph
import math
import numpy as np
import time

class TNT(nn.Module):
    r"""
     A class to reproduce TNT.
    """
    def __init__(self, feature_length=14, timeStampNumber=30, traj_num=1000, mid_traj_num=50, device='cuda:0', K=6):
        r"""
        Construct a TNT model to use vectornet to extract feature and generate multi-trajs
        :param feature_length: same as VectorNet.
        :param timeStampNumber: the length of time stamp for predicting the future trajectory.
        :param traj_num: the number of targets in first two stages
        :param final_traj_num: the number of trajctories in the last stage
        """
        super(TNT, self).__init__()
        self.device = device
        self.feature_length = feature_length
        self.timeStampNumber = timeStampNumber
        self.traj_num = traj_num
        self.mid_traj_num = mid_traj_num
        self.K = K
        self.hidden_size = 64
        self.teaching_force = 0.3
        self.vectorNet = VectorNet(feature_length=feature_length)
        self.car_feature = self.vectorNet.pLen #112
        # print(self.car_feature)
        self.target_pr = TargetPrediction(device, self.traj_num, self.mid_traj_num, self.car_feature)
        self.motion_est = Motion_est(device, self.teaching_force, self.timeStampNumber, self.car_feature, \
                                    self.feature_length, self.mid_traj_num)
        self.traj_score = traj_score(device, self.timeStampNumber, self.mid_traj_num, self.car_feature)
        self.subGraph_agent = SubGraph(layersNumber=3, feature_length=feature_length)
        self.node_complete = nn.Sequential(nn.Linear(self.vectorNet.pLen, self.hidden_size),
                                    nn.LayerNorm(self.hidden_size),
                                    nn.ReLU(True),
                                    nn.Linear(self.hidden_size, self.vectorNet.pLen))
        # self.node_complete_car = nn.Sequential(nn.Linear(self.vectorNet.pLen, self.hidden_size),
        #                             nn.LayerNorm(self.hidden_size),
        #                             nn.ReLU(True),
        #                             nn.Linear(self.hidden_size, self.vectorNet.pLen))
        self.node_loss = torch.nn.SmoothL1Loss()
    def forward(self, x, osm, osm_interval, labels, state='train'):
        r"""
        x: trajctory information
        osm: HD-map information
        osm_interval: length of each polyline
        """
        speed = x[:,0,-1,6:8].clone()
        agent_his_traj = x[:,0].to(self.device)
        agent_his_traj = self.subGraph_agent(agent_his_traj)
        if state == 'train':
            x, node_label = self.vectorNet(x, osm, osm_interval)
            recovered_node = self.node_complete(x)
            L_node = self.node_loss(recovered_node, node_label)
        else:
            x = self.vectorNet(x, osm, osm_interval, state)
        
        L_cls, L_off, top_targets = self.target_pr(osm, agent_his_traj, x, labels[:,-1,:], speed)
        
        L1 = L_cls + L_off
        if state == 'train':
            L1 += L_node
        pre, L_re = self.motion_est(top_targets, agent_his_traj, x, labels, labels[:,-1,:],state)
        L3, pre, dislist = self.traj_score(agent_his_traj, pre, labels, x)
        return L1, L_re, L3, pre, L_cls, dislist

class TargetPrediction(nn.Module):
    r"""
    This class stands for the first stage of TNT, to predict targets
    """        
    def __init__(self, device, init_target_num, out_tar_num, x_len):
        super(TargetPrediction, self).__init__()
        self.device = device
        self.init_target_num = init_target_num
        self.out_tar_num = out_tar_num
        self.x_len = x_len
        self.clsLoss = torch.nn.CrossEntropyLoss()
        self.offsetLoss = torch.nn.SmoothL1Loss()
        self.KL = torch.nn.KLDivLoss(reduction='batchmean')
        self.hidden = 64
        self.pi = torch.acos(torch.zeros(1)).item() * 2
        self.logsoftmax = nn.LogSoftmax(dim=1)
        self.temp = 0.01
        self.tem = 0.001
        self.small = 1e-6
        # self.target_proj = nn.Sequential(nn.Linear(2, self.hidden),
        #                                 nn.LayerNorm(self.hidden),
        #                                 nn.GELU(),
        #                                 nn.Linear(self.hidden, self.hidden))
        self.policy_dis = nn.Sequential(nn.Linear(self.x_len*2+2, self.hidden),
                                        nn.LayerNorm(self.hidden),
                                        nn.GELU(),
                                        nn.Linear(self.hidden, self.hidden),
                                        nn.LayerNorm(self.hidden),
                                        nn.GELU(),
                                        nn.Linear(self.hidden, 1))
        self.target_proj = nn.Sequential(nn.Linear(self.x_len*2+2, self.hidden),
                                        nn.LayerNorm(self.hidden),
                                        nn.GELU(),
                                        nn.Linear(self.hidden, self.hidden),
                                        nn.LayerNorm(self.hidden),
                                        nn.GELU(),
                                        nn.Linear(self.hidden, 2))
        self.offsetx_pre = nn.Sequential(nn.Linear(self.x_len*2+2, self.hidden),
                                        nn.LayerNorm(self.hidden),
                                        nn.GELU(),                                  
                                        nn.Linear(self.hidden, 2))
        self.offsety_pre = nn.Sequential(nn.Linear(self.x_len*2+2, self.hidden),
                                        nn.LayerNorm(self.hidden),
                                        nn.GELU(),
                                        nn.Linear(self.hidden, 1))
        # self.policy_fc = nn.Linear(self.hidden//4,1) 
        # self.offset_fc = nn.Linear(self.hidden//4,2)                               
    def forward(self, osm, agent_traj, vectornet_feature, end, speed):
        osm_points = osm[:,:,:2].clone()
        # print(osm_points.shape)
        if osm_points.shape[1] < self.init_target_num:
            osm_dis = osm_points.pow(2).sum(2).sqrt().unsqueeze(-1) #B * point_num * 1
            mask = (osm_dis>50).repeat(1,1,2)
            length = osm_points.shape[1]
            time = 4*torch.randn([osm_points.shape[0],osm_points.shape[1],1]).repeat(1,1,2).to(self.device).double()
            to_mask = time*torch.normal(speed.unsqueeze(1).repeat(1, length, 1)).to(self.device).double()
            osm_points[mask] = (to_mask)[mask]
            length = math.ceil(math.sqrt((self.init_target_num - osm_points.shape[1])/2))
            grid_x = torch.arange(-5, 10, 15/length).to(self.device).double()
            grid_y = torch.arange(-10, 10, 20/length).to(self.device).double()
            grids = []
            grids.append(osm_points)
            for i in range(length):
                newline = torch.zeros([osm.shape[0], len(grid_x), 2]).to(self.device).double()
                newline[:, :, 0] = grid_x.unsqueeze(0).repeat(osm.shape[0], 1)
                newline[:, :, 1] = grid_y[i]
                grids.append(newline)
            length = math.ceil((self.init_target_num - osm_points.shape[1])/2) + osm_points.shape[1]
            length = self.init_target_num-length
            time = torch.linspace(10, 35, length).reshape(1,length,1).repeat([osm.shape[0], 1, 1]).to(self.device).double()
            x = time*0.1*torch.normal(speed[:,0].unsqueeze(1).unsqueeze(1).repeat(1, length, 1)).to(self.device).double()
            y = time*0.1*torch.normal(speed[:,1].unsqueeze(1).unsqueeze(1).repeat(1, length, 1)).to(self.device).double()
            grids.append(torch.cat((x,y), dim=-1))
            osm_points = torch.cat(grids, dim=1)
            osm_points = osm_points[:, 0:self.init_target_num, :]
        else:
            osm_dis = osm_points.pow(2).sum(2).sqrt().unsqueeze(-1) #B * point_num * 1
            index = torch.argsort(osm_dis, dim=1)
            osm_points = torch.gather(osm_points,1,index.repeat(1,1,2))[:,:self.init_target_num]
            # pass #not needed to solve now
        agent_traj = agent_traj.unsqueeze(1).repeat(1,osm_points.shape[1],1)
        vectornet_feature = vectornet_feature.unsqueeze(1).repeat(1,osm_points.shape[1],1)
        # osm_points_emb = self.target_proj(osm_points)
        osmPlusfeature = torch.cat([vectornet_feature, agent_traj, osm_points],dim=-1)

        offset_x = self.offsetx_pre(osmPlusfeature)
        # offset_y = self.offsety_pre(osmPlusfeature)
        # offset_pr = torch.cat([offset_x, offset_y], dim=-1)
        targets = osm_points + offset_x
        # targets = self.offset_fc(targets)
        # targets_emb = self.target_proj(targets)
        # end_emb = self.target_proj(end)
        targetPlusfeature = torch.cat([vectornet_feature, agent_traj, targets], dim=-1)
        targets = self.target_proj(targetPlusfeature)
        # targetPlusfeature = torch.cat([vectornet_feature, agent_traj, osm_points], dim=-1)
        # policy_dist = self.policy_dis(targetPlusfeature) # 128* 1000 * 1
        # policy_dist = torch.softmax(policy_dist, dim=1)
        target_mean = torch.mean(targets,dim=1).unsqueeze(1).repeat(1,targets.shape[1],1)
        target_var = torch.var(targets,dim=1).unsqueeze(1).repeat(1,targets.shape[1],1)
        # # # # print(target_var.shape,target_mean.shape)
        # # policy_dist = torch.exp(-0.5*(policy_dist-target_mean).pow(2)/(target_var)+self.small)/torch.sqrt(2*self.pi*target_var+self.small)
        policy_dist_x = torch.exp(-0.5*(targets[:,:,0]-target_mean[:,:,0]).pow(2)/(target_var[:,:,0])+self.small)/torch.sqrt(2*self.pi*target_var[:,:,0]+self.small)
        policy_dist_y = torch.exp(-0.5*(targets[:,:,1]-target_mean[:,:,1]).pow(2)/(target_var[:,:,1])+self.small)/torch.sqrt(2*self.pi*target_var[:,:,1]+self.small)
        policy_dist = (policy_dist_x*policy_dist_y) #.unsqueeze(-1)
        # policy_dist_log = self.logsoftmax(policy_dist*5)
        # print(policy_dist.shape)
        L_off = self.offsetLoss(targets, end.unsqueeze(1).repeat(1,osm_points.shape[1],1))
        dis = (osm_points - end.unsqueeze(1).repeat(1,osm_points.shape[1],1)).pow(2).sum(2).sqrt()
        dis = torch.softmax(-dis/self.temp,dim=1)
        # L_cls = self.KL(policy_dist_log.squeeze(), dis)

        target_index = torch.argmax(dis, dim=1)
        L_cls = self.clsLoss(policy_dist, target_index)
        # L_cls = -torch.log(torch.gather(policy_dist,1,target_index.unsqueeze(-1).unsqueeze(-1).repeat(1,1000,1))[:,0]).mean()
        p_index = torch.argsort(policy_dist.unsqueeze(-1), dim=1, descending=True)
        # print(p_index.shape)
        # top_targets_emb = torch.gather(targets_emb, 1, p_index.repeat(1,1,targets_emb.shape[2]))[:,0:self.out_tar_num,:]
        top_targets = torch.gather(targets, 1, p_index.repeat(1,1,targets.shape[2]))[:,0:self.out_tar_num,:]
        L_end = self.offsetLoss(top_targets.clone(), end.unsqueeze(1).repeat(1,top_targets.shape[1],1))
        # print(L_off.item(), ' L_cls: ', L_cls.item())
        return L_cls, L_off + L_end*3, top_targets
    
class Motion_est(nn.Module):
    r"""
    This part stands for the second stage of TNT, with targets in hand, predict trajectory
    basically, it is just use the tagets from last state to predict a trajectory, no more
    """
    def __init__(self, device, teaching_force, timeStampNumber, x_len, feature_length, traj_num):
        super(Motion_est, self).__init__()
        self.teaching_force = teaching_force
        self.device = device
        self.timeStamp = timeStampNumber
        self.hidden_size = 64
        self.x_len = x_len
        self.traj_num = traj_num
        self.car_feature = x_len
        self.K = 30
        self.trajDecoder = nn.Sequential(nn.Linear(self.x_len + self.car_feature + 2, self.hidden_size),
                                    nn.LayerNorm(self.hidden_size),
                                    nn.ReLU(True),
                                    nn.Linear(self.hidden_size, self.hidden_size),
                                    nn.LayerNorm(self.hidden_size),
                                    nn.ReLU(True),
                                    nn.Linear(self.hidden_size, timeStampNumber * 2))
        self.Ls2 = torch.nn.SmoothL1Loss()
    def forward(self, targets, agent_traj, vectornet_feature, labels, end, state='train'):
        mask = torch.rand([targets.shape[0], targets.shape[1]])
        if state == 'train':
            mask = (mask < self.teaching_force)
        else:
            mask = (mask < 0)
        end = end.unsqueeze(1).repeat(1,targets.shape[1],1)
        # end_emb = end_emb.unsqueeze(1).repeat(1,targets.shape[1],1)
        # targets_emb[mask] = end_emb[mask]
        # L_end = self.Ls2(end, targets)
        targets[mask] = end[mask]
        # print(targets[0])

        targetPlusfeature = torch.cat([vectornet_feature.unsqueeze(1).repeat(1,targets.shape[1],1), targets],
                                        dim=-1)
        targetPlusfeature = torch.cat([targetPlusfeature, agent_traj.unsqueeze(1).repeat(1,targets.shape[1],1)],dim=-1)
        
        pre = self.trajDecoder(targetPlusfeature)
        x = pre.clone() 
        x = x.reshape([x.shape[0], targets.shape[1], self.timeStamp, 2])
        L_end = self.Ls2(x[:,:,-1], targets)
        
        # print(labels.shape)
        # labels = labels.unsqueeze(1).repeat(1, targets.shape[1], 1, 1)
        x_after = x.clone()[:,:,1:]
        x_before = x.clone()[:,:,:-1] 
        x_between = x_after - x_before

        label_after = labels.unsqueeze(1).repeat(1, targets.shape[1], 1, 1)[:,:,1:]
        label_before = labels.unsqueeze(1).repeat(1, targets.shape[1], 1, 1)[:,:,:-1]
        label_between = label_after - label_before

        L_between = self.Ls2(x_between, label_between)

        L_re_list = []
        # L_be_list = []
        # L_re = self.Ls2(x, labels)
        L_re = 0
        for i in range(targets.shape[1]):
            # L_be_list.append(self.Ls2(x_between[:,i], label_between[:,i]))
            L_re_list.append(self.Ls2(x[:,i], labels))
        # L_between = 0
        for i in range(self.K):
            L_min = min(L_re_list)
            L_re += L_min
            L_re_list.remove(L_min)
        # L_re = self.Ls2(x, labels.unsqueeze(1).repeat(1, targets.shape[1], 1, 1))
        return pre, L_re + L_between*3 + L_end

class traj_score(nn.Module):
    def __init__(self, device, timeStampNumber, traj_num, x_len, K=6):
        super(traj_score, self).__init__()
        self.device = device 
        self.timeStampNumber = timeStampNumber
        self.traj_num = traj_num
        self.output_num = K
        self.hidden = 64
        self.x_len = x_len
        self.pi = torch.acos(torch.zeros(1)).item() * 2
        self.small = 1e-6
        # self.only_sort = 3
        self.threshold = 0.1
        self.clsLoss = torch.nn.CrossEntropyLoss()
        self.KL = torch.nn.KLDivLoss(reduction='batchmean')
        self.logsoftmax = nn.LogSoftmax(dim=1)
        self.g = nn.Sequential(nn.Linear(self.x_len*2+2*self.timeStampNumber, self.hidden),
                                        nn.LayerNorm(self.hidden),
                                        nn.GELU(),
                                        nn.Linear(self.hidden, self.hidden),
                                        nn.LayerNorm(self.hidden),
                                        nn.GELU(),
                                        nn.Linear(self.hidden, 1))
        self.alpha = 0.01
    def CrossEntropy(self, pred, targets):
        logSoftMax = nn.LogSoftmax(dim=1)
        return torch.mean(torch.sum(-targets * logSoftMax(pred), 1))
    
    def forward(self, agent_traj, pre, labels, vectornet_feature):
        vectornet_feature = vectornet_feature.unsqueeze(1).repeat(1, pre.shape[1], 1)
        agent_traj = agent_traj.unsqueeze(1).repeat(1, pre.shape[1], 1)
        # print(vectornet_feature.shape)
        vectornet_feature = torch.cat([vectornet_feature, agent_traj, pre], dim=-1)

        pre_distribution = self.g(vectornet_feature).reshape(vectornet_feature.shape[0],-1)
        # tar_mean = torch.mean(pre_distribution,dim=1).unsqueeze(1).repeat(1, pre_distribution.shape[1])
        # tar_var = torch.var(pre_distribution,dim=1).unsqueeze(1).repeat(1, pre_distribution.shape[1])
        # pre_distribution = torch.exp(-0.5*(pre_distribution-tar_mean).pow(2)/(tar_var+self.small))/torch.sqrt(2*self.pi*tar_var+self.small)
        pre_distribution = self.logsoftmax(pre_distribution) # B * M

        labels = labels.unsqueeze(1).repeat(1, pre.shape[1], 1, 1)
        pre = pre.reshape([pre.shape[0], pre.shape[1], self.timeStampNumber, 2])
        dis = (pre - labels).pow(2).sum(dim=-1).squeeze() #expected B * M * seq_len

        dis = -torch.max(dis, dim = -1)[0]/self.alpha #B * M
        dis = torch.softmax(dis, dim=-1)
        # target_index = torch.argmin(dis, dim=1)
        L3 = self.KL(pre_distribution, dis)
        
        # L3 = self.CrossEntropy(pre_distribution, dis)
        pre_index = torch.argsort(pre_distribution, dim=1, descending=True).unsqueeze(-1).unsqueeze(-1).repeat(1, 1, self.timeStampNumber, 2)
        pre = torch.gather(pre, 1, pre_index)
        # output = pre[:,:self.output_num]
        # output = torch.zeros_like(pre[:,:self.output_num])
        # print(torch.mean(tmp[:,:10]))
        top_list = []
        top_dis_list = []
        i = 0
        pre_distrt = pre_distribution.clone()
        # back_pre = pre.clone()
        while len(top_list)<self.output_num:
            if -1 in pre_distrt[:,0]:
                # print(len(top_list),", empty!")
                break
            top = pre[:, 0].unsqueeze(1)
            top_list.append(top)
            top_dis_list.append(torch.exp(pre_distrt[:,0].unsqueeze(1)))
            # top = pre[:, 0].unsqueeze(1).repeat(1,pre.shape[1]-1,1,1)
            pre = pre[:, 1:]
            pre_distrt = pre_distrt[:,1:]

            mask = (pre[:, :, 0].pow(2).sum(dim=-1).sqrt()>5)
            pre_distrt[mask] = -1
            dist = []
            for top in top_list:
                tmp = pre - top.repeat(1,pre.shape[1],1,1)
                tmp = tmp.pow(2).sum(dim=-1).sqrt()
                dist.append(tmp.unsqueeze(-1))
            dist = torch.cat(dist,dim=-1)
            dist = torch.mean(dist,dim=-1)
            tmp = torch.max(dist, dim = -1)[0]
            mask = (tmp < self.threshold)
            pre_distrt[mask] = 1.1*pre_distrt[mask]
            tmp_index = torch.argsort(pre_distrt, dim=1, descending=True)
            pre = torch.gather(pre, 1, tmp_index.unsqueeze(-1).unsqueeze(-1).repeat(1,1,self.timeStampNumber,2))
            pre_distrt = torch.gather(pre_distrt, 1, tmp_index)
            i += 1 
            # print(tmp.shape) 
        i = 1
        while len(top_list)<self.output_num:
            if i > pre.shape[1]-1:
                print("damn, not enough!")
            candidate = pre[:, i].unsqueeze(1)
            # print(candidate.shape)
            top_list.append(candidate)
            i += 1
        output = torch.cat(top_list,dim=1)
        dislist = torch.cat(top_dis_list,dim=1)
        # print(output.shape)
        return L3, output, dislist
"""
task design:
soft body support,
dynamic task: dynamic grasping
better learning

Benchmark design: 
why the benchmark work
Behaviour suite for reinforcement learning

sota only solve three
how to solve nine benchmark
"""















        



