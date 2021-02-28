import torch
from torch import nn
import torch.nn.functional as F
from predictors.sub_graph import SubGraph
from predictors.global_graph import Attention
import time
import random

class VectorNet(nn.Module):

    r"""
    Vector network. 
    """

    # def __init__(self, len, pNumber):
    def __init__(self, feature_length, device='cpu'):
        r"""
        Construct a VectorNet.
        :param feature_length: length of each vector v ([ds,de,a,j]).
        """
        super(VectorNet, self).__init__()
        layersNumber = 3
        # self.subGraphs = clones(SubGraph(layersNumber=3, feature_length=feature_length), 3)
        self.subGraphs = SubGraph(layersNumber=layersNumber, feature_length=feature_length)
        # self.pLen = feature_length
        self.pLen = feature_length * (2 ** layersNumber)
        self.globalGraph = Attention(C=self.pLen)
        self.device = device
    def forward(self, data, osm, osm_interval, state='train'):
        r"""

        :param data: the trajectory of all road agents
              shape: data.shape = [batch size, vNumber, feature_length]
        :return: output
        """
        nCar = 10
        # osm map
        osm_subGraph_list = []
        # Suppose we have 10 cars in each frame, then n = 10.
        # all_batch_nodes = [] #[batch, nCar, node_feature]
        # global_graphs = [] #[batch, features]
        # for i in range(len(data)): # num of batch
        #     data1 = data[i]
        #     each_batch_nodes = [] # in different batch: different len
        #     for j in range(len(data1)): # num of trajs(cars that appear), traj0 = agent
        #         sub_node = self.subGraphs(data1[j].to(self.device).double().unsqueeze(0))
        #         # print(sub_node.shape)
        #         each_batch_nodes.append(sub_node)
        #         # break
        #     each_batch_nodes = torch.cat(each_batch_nodes, dim=0) #[numCar, node_features]
        #     each_batch_nodes_with_osm = torch.cat([each_batch_nodes, osm_nodes[i,:,:]], dim=0) #[numCar+numRoad, features]
        #     global_graphs.append(self.globalGraph(each_batch_nodes_with_osm.unsqueeze(0)))
        # if state == 'train':
        #     mask = random.randint(0, len(osm_interval)-2)
        #     for i in range(len(osm_interval)-1):
        #         if i != mask:
        #             osm_subGraph_list.append(self.subGraphs(osm[:, osm_interval[i]:osm_interval[i+1], :]).unsqueeze(1))
        #         elif i == mask:
        #             true_feature = self.subGraphs(osm[:, osm_interval[i]:osm_interval[i+1], :]).unsqueeze(1)
        #             masked_feature = torch.zeros_like(osm[:, osm_interval[i]:osm_interval[i+1], :])
        #             masked_feature[:, :, :2] = osm[:, osm_interval[i]:osm_interval[i+1], :2]
        #             osm_node = self.subGraphs(masked_feature.to(self.device).double()).unsqueeze(1)
        #             osm_subGraph_list.append(osm_node)
        # else:
        for i in range(len(osm_interval)-1):
            osm_subGraph_list.append(self.subGraphs(osm[:, osm_interval[i]:osm_interval[i+1], :]).unsqueeze(1))

        osm_nodes = torch.cat(osm_subGraph_list, dim=1) #[batch, road_poly_num, features]

        trajs = []
        if state == 'train':
            mask = random.randint(0, data.shape[1]-1)
            for i in range(data.shape[1]):
                if i != mask:
                    sub_traj = self.subGraphs(data[:,i].to(self.device).double()).unsqueeze(1) # 128*1*112
                    trajs.append(sub_traj)
                elif i == mask:
                    true_feature = self.subGraphs(data[:,i].to(self.device).double())
                    masked_feature = torch.zeros_like(data[:,i])
                    masked_feature[:, :2] = data[:, i, :2]
                    sub_traj = self.subGraphs(masked_feature.to(self.device).double()).unsqueeze(1)
                    trajs.append(sub_traj)

            trajs = torch.cat(trajs, dim=1)
            traj_osm = torch.cat([trajs, osm_nodes], dim=1)
            traj_osm = F.normalize(traj_osm, dim=-1, p=2)
            global_graphs = self.globalGraph(traj_osm)
            # true_feature = torch.cat([true_feature, true_car_feat], dim=1)
            true_feature = F.normalize(true_feature, dim=-1, p=2)

            return global_graphs, true_feature
        else:
            for i in range(data.shape[1]):
                sub_traj = self.subGraphs(data[:,i].to(self.device).double()).unsqueeze(1)
                trajs.append(sub_traj)
            trajs = torch.cat(trajs, dim=1)
            traj_osm = torch.cat([trajs, osm_nodes], dim=1)
            traj_osm = F.normalize(traj_osm, dim=-1, p=2)
            global_graphs = self.globalGraph(traj_osm)
            return global_graphs

class VectorNetWithPredicting(nn.Module):

    r"""
      A class for packaging the VectorNet and future trajectory prediction module.
      The future trajectory prediction module uses MLP without ReLu(because we
    hope the coordinate of trajectory can be negative).
    """

    def __init__(self, feature_length, timeStampNumber, device='cpu'):
        r"""
        Construct a VectorNet with predicting.
        :param feature_length: same as VectorNet.
        :param timeStampNumber: the length of time stamp for predicting the future trajectory.
        """
        super(VectorNetWithPredicting, self).__init__()
        self.device = device
        self.vectorNet = VectorNet(feature_length=feature_length)
        self.timeStamp = timeStampNumber
        self.hidden_size = 64
        self.car_feature = self.vectorNet.pLen #14
        self.trajDecoder = nn.Sequential(nn.Linear(self.vectorNet.pLen + self.car_feature, self.hidden_size),
                                    nn.LayerNorm(self.hidden_size),
                                    nn.ReLU(True),
                                    nn.Linear(self.hidden_size, self.hidden_size),
                                    nn.LayerNorm(self.hidden_size),
                                    nn.ReLU(True),
                                    nn.Linear(self.hidden_size, timeStampNumber * 2))
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
         #MLP.MLP(inputSize=self.vectorNet.pLen,outputSize=timeStampNumber * 2,noReLU=False)


    def forward(self, x, osm, osm_interval, state='train'):
        r"""

        :param x: the same as VectorNet.
        :return: Future trajectory vector with length timeStampNumber*2, the form is (x1,y1,x2,y2,...).
        """
        # agent_his_traj = x[:,9,:].clone() # the number indicates last known frame
        # agent_his_traj[:,-1] = 0.0
        # agent_his_batches = []
        # for i in range(len(x)):
        #     agent_his_batches.append(x[i][0][:])
        agent_his_traj = x[:,0].to(self.device).double()
        agent_his_traj = self.subGraph_agent(agent_his_traj)
        # print(agent_his_traj.shape)
        # agent_his_traj = 
        if state == "train":
            x, node_label = self.vectorNet(x, osm, osm_interval)
            # recovered_node_car = self.node_complete_car(x).unsqueeze(1)
            recovered_node = self.node_complete(x)
            # recovered_node = torch.cat([recovered_node,recovered_node_car], dim=1)
            recovered_node = recovered_node.reshape(node_label.shape)
            # print(node_label.shape)
            L_node = self.node_loss(recovered_node, node_label)
            x = torch.cat([x, agent_his_traj], dim=1)
            x = self.trajDecoder(x)
            x = x.reshape([x.shape[0], self.timeStamp, 2])
            return x, L_node
        else:
            x = self.vectorNet(x, osm, osm_interval, state)
            x = torch.cat([x, agent_his_traj], dim=1)
            x = self.trajDecoder(x)
            x = x.reshape([x.shape[0], self.timeStamp, 2])
            return x


class VectorNetAndTargetPredicting(nn.Module):
    def __init__(self, feature_length):
        super(VectorNetAndTargetPredicting, self).__init__()
        self.vectornet = VectorNet(feature_length=feature_length)
        self.car_feature = 14
        self.hidden_size = 150
        self.targetPred = nn.Sequential(nn.Linear(self.vectornet.pLen + self.car_feature, self.hidden_size),
                                        nn.LayerNorm(self.hidden_size),
                                        nn.ReLU(True),
                                        nn.Linear(self.hidden_size, self.hidden_size),
                                        nn.LayerNorm(self.hidden_size),
                                        nn.ReLU(True),
                                        nn.Linear(self.hidden_size, 2)) # output (x,y)
    def forward(self, x):
        agent_his_traj = x[:,19,:].clone()
        agent_his_traj[:,-1] = 0.0
        x = self.vectornet(x)
        x = torch.cat([x, agent_his_traj], dim=1)
        x = self.targetPred(x)

        return x