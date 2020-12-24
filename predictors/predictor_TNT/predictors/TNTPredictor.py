import math
import torch
import torch.nn as nn
import torch.nn.functional as F
from predictors.extract_osm import *
from predictors.sub_graph import *
from predictors.TNT import *
import logging

import os
import logging
import numpy as np

#from traj import *
#from predictor import Predictor, MyState
# Attention! Because I want to test it locally, the above two lines I did a slight change to make sure the code can work without the simulator
# the original code looks like that, hope you can see. With cuda:0, this version of code can surely work with the dir to map is correct
#because in the predictor it only test one single case, I write the code exactly works for that scenario.
from predictors.traj import *
from predictors.predictor import Predictor, MyState

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("predictor")

def RotateOsm(data, R):
    R = R.unsqueeze(1)
    R = R.repeat(1,data.shape[1],1,1)
    all_starts = data[:,:,:2].clone()
    all_ends = data[:,:,2:4].clone()
    all_starts = torch.cat([all_starts, torch.ones_like(data[:,:,:1])], dim=-1)
    all_ends = torch.cat([all_ends, torch.ones_like(data[:,:,:1])], dim=-1)
    all_starts = torch.matmul(R, all_starts.unsqueeze(-1)).squeeze(-1)
    all_ends = torch.matmul(R, all_ends.unsqueeze(-1)).squeeze(-1)

    all_starts = all_starts[:,:,:2]
    all_ends = all_ends[:,:,:2]

    data[:,:,:2] = all_starts
    data[:,:,2:4] = all_ends
    return data
def Local2Global_whole(iR, locs):
    if len(locs.shape) == 4:
        pNum = locs.shape[2]
        tNum = locs.shape[1]
        
        iR = iR.unsqueeze(1).unsqueeze(1)
        # print(iR.shape)
        iR = iR.repeat(1, tNum, pNum, 1, 1)

        locs = torch.cat([locs, torch.ones_like(locs[:,:,:,:1])], dim = -1)
        loc_global = torch.matmul(iR, locs.unsqueeze(-1)).squeeze(-1)
        loc_global = loc_global[:,:,:,:2]
    else:
        pNum = locs.shape[1]
        iR = iR.unsqueeze(1)
        # print(iR.shape)
        iR = iR.repeat(1, pNum, 1, 1)

        locs = torch.cat([locs, torch.ones_like(locs[:,:,:1])], dim = -1)
        loc_global = torch.matmul(iR, locs.unsqueeze(-1)).squeeze(-1)
        loc_global = loc_global[:,:,:2]
    return loc_global

def proj_mat_inverse(theta, X, Y):
    X = np.array(X)
    Y = np.array(Y)
    theta = np.array(theta)
    X = np.expand_dims(X, axis=(0, 1, 2))
    Y = np.expand_dims(Y, axis=(0, 1, 2))
    R11 = np.expand_dims(np.cos(theta), axis=(0, 1, 2))
    R12 = np.expand_dims(-np.sin(theta), axis=(0, 1, 2))
    R21 = np.expand_dims(np.sin(theta), axis=(0, 1, 2))
    R22 = np.expand_dims(np.cos(theta), axis=(0, 1, 2))

    R1 = np.concatenate((R11, R12, X), axis=2)
    R2 = np.concatenate((R21, R22, Y), axis=2)
    R3 = np.zeros([X.shape[0], 1, 3])
    R3[:, :, 2] = 1
    R = np.concatenate([R1, R2, R3], axis=1)
    return R


def proj_mat(R):
    R_inversed = R.copy().transpose((0, 2, 1))
    R_inversed[:, 2, :] = np.array([0.0, 0.0, 1.0])
    R_inversed[:, 0, 2] = -R_inversed[:, 0, 0] * \
        R[:, 0, 2] - R_inversed[:, 0, 1]*R[:, 1, 2]
    R_inversed[:, 1, 2] = -R_inversed[:, 1, 0] * \
        R[:, 0, 2] - R_inversed[:, 1, 1]*R[:, 1, 2]
    return R_inversed

class TNTPredictor(Predictor):
    def __init__(self, logger: logging.Logger):
        super().__init__()

        self._logger = logger
        self.args = {
            'load': 'TNT_dict.pt',
        }
        print('args', self.args)

        self.dev = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        print('dev', self.dev)
        self.model = TNT(feature_length=14, timeStampNumber=30).double()
        self.model.to(self.dev)
        self.model.load_state_dict(torch.load(os.path.join(os.path.dirname(__file__), 'TNT_dict.pt'), map_location=self.dev))
        self.model.eval()

        ##########################################

        self.last_state = None
        self.results = None
    def start(self):
        pass

    def shutdown(self):
        pass
    def SplitOsm(self, data):
        pids = data[:,-1].clone()
        osm_interval = [0]
        last_pid = pids[0]
        for i in range(pids.shape[0]-1):
            if last_pid != pids[i]:
                last_pid = pids[i]
                osm_interval.append(i)
        osm_interval.append(pids.shape[0])
        data[:,-1] = 0.0
        return osm_interval, data
    def on_env(self, map_name, my_traj: Trajectory, other_trajs: []):
        assert self.last_state is None
        assert self.results is None

        #############################################
        osm_base = os.path.join(os.path.dirname(__file__), 'Maps')
        osm_path = os.path.join(osm_base, map_name) + '.osm'
        
        self.map_data = torch.from_numpy(main_vector(osm_path)).double().squeeze()
        self.map_interval, self.map_data = self.SplitOsm(self.map_data)
        self.map_data = self.map_data.unsqueeze(0) #1*poliline_num*14

        self._logger.info(f'predictor: Receive from Simulator')
        assert len(my_traj.state()) == 10, 'The length of the historical trajectory must be 10'

        self._logger.info(f'map_name: {map_name}; my_id: {my_traj.state()[0].track_id}; other_trajs size: {len(other_trajs)}')

        his = []
        for i, s in enumerate(my_traj.state()):
            #self._logger.info(f'frame_id: {s.frame_id}; x: {s.x}; y: {s.y}; jaw: {s.psi_rad}')
            his.append([s.track_id, s.x, s.y, s.vx, s.vy,\
                        int(s.agent_type != 'car'), s.length, s.width, s.psi_rad, 1, i]) 
                        #1 in the end means it is the agent, frame_id is supposed to be from 0-10
            self.last_state = s
        other_traj_features = []
        if len(other_trajs) != 0:
            for traj in other_trajs:
                traj_feature = []
                for i, s in enumerate(traj.state()):
                    traj_feature.append([s.track_id, s.x, s.y, s.vx, s.vy,\
                                        int(s.agent_type != 'car'), s.length, s.width, s.psi_rad, 0, i])
                other_traj_features.append(traj_feature.copy())
        his = torch.tensor(his) #10,11
        
        if len(other_trajs) != 0:
            other = torch.tensor(other_traj_features) #car_num-1,10,10
            trajs = torch.cat([his.unsqueeze(0),other],dim=0) #car_num, 10, 10(feature_dim)
        else:
            trajs = his.unsqueeze(0) #1,10,10
        theta = his[-1,8]
        #get transform matrix
        #print(theta, his[9][1], his[9][2])
        iR = proj_mat_inverse(theta, his[9][1], his[9][2])
        R = proj_mat(iR)
        # print("before")
        # print(trajs[0,:,1:5])
        for i in range(trajs.shape[0]): #transform  traj coordinate
            track_tmp = trajs[i, :, 1:5].clone()
            trajs[i, :, 1] = R[0, 0, 0]*track_tmp[:, 0] + \
                R[0, 0, 1]*track_tmp[:, 1] + R[0, 0, 2]
            trajs[i, :, 2] = R[0, 1, 0]*track_tmp[:, 0] + \
                R[0, 1, 1]*track_tmp[:, 1] + R[0, 1, 2]
            trajs[i, :, 3] = R[0, 0, 0]*track_tmp[:, 2] + \
                R[0, 0, 1]*track_tmp[:, 3]
            trajs[i, :, 4] = R[0, 1, 0]*track_tmp[:, 2] + \
                R[0, 1, 1]*track_tmp[:, 3]
        # print(trajs[0,:,1:5])
        bu = torch.zeros_like(trajs[:,:,:3])
        car_traj = torch.cat([trajs.clone(),bu],dim=-1)
        car_traj[:,:,0] = trajs[:,:,1] #start xs
        car_traj[:,:,1] = trajs[:,:,2] #start ys
        car_traj[:,:,2] = trajs[:,:,1] + trajs[:,:,3]*0.1 #end xs of a vector
        car_traj[:,:,3] = trajs[:,:,2] + trajs[:,:,4]*0.1 #end ys of a vector
        car_traj[:,:,4] = torch.ones_like(car_traj[:,:,4]) # 1 is car, 0 is map
        car_traj[:,:,5] = trajs[:,:,-1] * 0.1 #frame id
        car_traj[:,:,6:13] = trajs[:,:,3:10] #other original features
        car_traj[:,:,-1] = torch.zeros_like(trajs[:,:,0])#clean track id

        car_traj = car_traj.unsqueeze(0).to(self.dev) #1*car_num*10*14
        iR = torch.from_numpy(iR).double() #[1,3,3]
        R = torch.from_numpy(R).double()
        self.map_data = RotateOsm(self.map_data, R).to(self.dev).double()
        test_data = [car_traj, self.map_data, self.map_interval, iR]
        _, _, _, preds, _, dislist = self.model(test_data[0].double(), test_data[1].double(), test_data[2], torch.zeros([1,30,2]).to(self.dev).double(), state='test')
        preds = Local2Global_whole(iR.to(self.dev), preds)
        #########################################
        self.results = ((preds.squeeze()).cpu().detach().numpy().copy(),dislist)

    def fetch_my_state(self) -> MyState:
        assert self.last_state is not None
        assert self.results is not None

        ##################################################

        self._logger.info(f'predictor: Echo results back to simulator\n')
        dislist = self.results[1]
        self.results = self.results[0]
        trajs = []
        for j in range(len(self.results)):
            traj = Trajectory()
            for i in range(30):
                s = State()

                s.track_id = self.last_state.track_id
                s.frame_id = self.last_state.frame_id + i + 1
                s.timestamp_ms = s.frame_id * 100
                s.agent_type = self.last_state.agent_type
                s.x = self.results[j][i][0]
                s.y = self.results[j][i][1]
                s.vx = ((self.results[j][i][0] - self.results[j][i - 1][0]) if i > 0 else self.results[j][i][0] - self.last_state.x) * 10
                s.vy = ((self.results[j][i][1] - self.results[j][i - 1][1]) if i > 0 else self.results[j][i][1] - self.last_state.y) * 10
                
                #print(s.x,s.y, s.vx, s.vy)
                
                s.psi_rad = math.atan2(s.vy, s.vx)
                s.length = self.last_state.length
                s.width = self.last_state.width
                #TODO: lanelet

                traj.append_state(s)
            
            #print()
            assert len(traj.state()) == 30, len(traj.state())
            trajs.append(traj)

        self.last_state = None
        self.results = None

        #print('dislist', dislist)

        dislist = dislist.squeeze()
        dislist = [x.item() for x in dislist]

        normalization = sum(dislist)
        dislist = [x / normalization for x in dislist]
        #print('dislist', dislist)

        assert len(trajs) == len(dislist)
        res = MyState(trajs, dislist)
        return res


'''
if __name__ == "__main__":
    predictor = VectornetPredictor(logger)
    my_traj = Trajectory()
    s0 = State()
    s0.track_id = 1
    s0.frame_id = 0
    s0.timestamp_ms = 100
    s0.agent_type = 'car'
    s0.x = 965.783
    s0.y = 988.577
    s0.vx = -6.7
    s0.vy = 0.492
    s0.psi_rad = 3.068
    s0.length = 4.15
    s0.width = 1.72
    my_traj.append_state(s0)
    s1 = State()
    s1.track_id = 1
    s1.frame_id = 1
    s1.timestamp_ms = 200
    s1.agent_type = 'car'
    s1.x = 965.113
    s1.y = 988.626
    s1.vx = -6.701
    s1.vy = 0.489
    s1.psi_rad = 3.069
    s1.length = 4.15
    s1.width = 1.72
    my_traj.append_state(s1)
    s2 = State()
    s2.track_id = 1
    s2.frame_id = 2
    s2.timestamp_ms = 300
    s2.agent_type = 'car'
    s2.x = 964.443
    s2.y = 988.674
    s2.vx = -6.692
    s2.vy = 0.485
    s2.psi_rad = 3.069
    s2.length = 4.15
    s2.width = 1.72
    my_traj.append_state(s2)
    s3 = State()
    s3.track_id = 1
    s3.frame_id = 3
    s3.timestamp_ms = 300
    s3.agent_type = 'car'
    s3.x = 963.773
    s3.y = 988.722
    s3.vx = -6.67
    s3.vy = 0.48
    s3.psi_rad = 3.069
    s3.length = 4.15
    s3.width = 1.72
    my_traj.append_state(s3)
    s4 = State()
    s4.track_id = 1
    s4.frame_id = 4
    s4.timestamp_ms = 300
    s4.agent_type = 'car'
    s4.x = 963.106
    s4.y = 988.77
    s4.vx = -6.634
    s4.vy = 0.474
    s4.psi_rad = 3.067
    s4.length = 4.15
    s4.width = 1.72
    my_traj.append_state(s4)
    s5 = State()
    s5.track_id = 1
    s5.frame_id = 5
    s5.timestamp_ms = 300
    s5.agent_type = 'car'
    s5.x = 962.443
    s5.y = 988.816
    s5.vx = -6.583
    s5.vy = 0.467
    s5.psi_rad = 3.071
    s5.length = 4.15
    s5.width = 1.72
    my_traj.append_state(s5)
    s6 = State()
    s6.track_id = 1
    s6.frame_id = 6
    s6.timestamp_ms = 300
    s6.agent_type = 'car'
    s6.x = 961.784
    s6.y = 988.862
    s6.vx = -6.518
    s6.vy = 0.458
    s6.psi_rad = 3.071
    s6.length = 4.15
    s6.width = 1.72
    my_traj.append_state(s6)
    s7 = State()
    s7.track_id = 1
    s7.frame_id = 7
    s7.timestamp_ms = 300
    s7.agent_type = 'car'
    s7.x = 961.133
    s7.y = 988.907
    s7.vx = -6.438
    s7.vy = 0.449
    s7.psi_rad = 3.072
    s7.length = 4.15
    s7.width = 1.72
    my_traj.append_state(s7)
    s8 = State()
    s8.track_id = 1
    s8.frame_id = 8
    s8.timestamp_ms = 300
    s8.agent_type = 'car'
    s8.x = 960.489
    s8.y = 988.952
    s8.vx = -6.345
    s8.vy = 0.439
    s8.psi_rad = 3.072
    s8.length = 4.15
    s8.width = 1.72
    my_traj.append_state(s8)
    s9 = State()
    s9.track_id = 1
    s9.frame_id = 9
    s9.timestamp_ms = 300
    s9.agent_type = 'car'
    s9.x = 959.854
    s9.y = 988.995
    s9.vx = -6.241
    s9.vy = 0.429
    s9.psi_rad = 3.073
    s9.length = 4.15
    s9.width = 1.72
    my_traj.append_state(s9)
    other_traj = Trajectory()
    for i in range(10):
        s11= State()
        s11.track_id = 2
        s11.frame_id = i
        s11.timestamp_ms = (i+1)*100
        s11.agent_type = 'car'
        s11.x = 1003
        s11.y = 987.4
        s11.vx = 0
        s11.vy = 0
        s11.psi_rad = 3.12
        s11.length = 4.69
        s11.width = 1.79
        other_traj.append_state(s11)
    other_trajs = [other_traj]
    predictor.on_env("DR_USA_Intersection_EP0.osm",my_traj, other_trajs)
    res = predictor.fetch_my_state()
    # print(res.trajectories[0])
'''



