# By Yaofeng Sun, Nov. 15, 2020

import math
import torch
import torchvision

import os
import logging
import numpy as np
from cv2 import cv2

from predictors.traj import *
from predictors.predictor import Predictor, MyState
from predictors.modules import ModelX, SampleTraj
from predictors.config import *



class CnnRnnPredictor(Predictor):

    def __init__(self, logger: logging.Logger):
        super().__init__()

        self._logger = logger
        self.args = {
            'load' : 'best_model.pt',
            'n_gaussians': 5,
            'n_samples': 1,
            'encoder': 'gru',
            'decoder': 'mdn',
            'K': 2,
            'd_h': 128,
        }
        print('args', self.args)

        self.dev = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        print('dev', self.dev)

        ##########################################

        self.model = ModelX(self.args['encoder'], self.args['decoder'], self.args['K'], self.args['d_h'], self.args['n_gaussians'], self.dev)
        self.model.to(self.dev)

        self.model.load_state_dict(torch.load(os.path.join(os.path.dirname(__file__), 'best_model.pt'), map_location=self.dev))
        self.model.eval()

        self.resnet = torchvision.models.resnet18(pretrained=True)
        self.resnet.to(self.dev)

        for parma in self.resnet.parameters():
            parma.requires_grad = False

        self.resnet.eval()

        ##########################################

        self.last_state = None
        self.results = None

    def start(self):
        pass

    def shutdown(self):
        pass
    
    def coord_convert(self, x, y):
        x = np.clip(int(x * 5), -110, +110) + 112
        y = np.clip(int(y * 5), -110, +110) + 112
        return x, y

    def on_env(self, map_name, my_traj: Trajectory, other_trajs: []):
        assert self.last_state is None, self.last_state
        assert self.results is None, self.results

        self._logger.info(f'predictor: Receive from Simulator')
        self._logger.info(f'map_name: {map_name}; other_trajs size: {len(other_trajs)}')

        assert len(my_traj.state()) == T_in, 'The length of the historical trajectory must be %d' % T_in
        for state in my_traj.state():
            #self._logger.info(f'frame_id: {state.frame_id}; x: {state.x}; y: {state.y}; jaw: {state.psi_rad}')
            self.last_state = state

        #############################################

        start_point = np.array([my_traj.state()[0].x, my_traj.state()[0].y])
        #print('start_point', start_point)

        # Generate historical input
        his = []
        for t in range(T_in):
            agent_t = my_traj.state()[t]
            his.append([agent_t.x - start_point[0], agent_t.y - start_point[1], agent_t.vx, agent_t.vy])

        his = np.array(his).astype(np.float32)
        his = torch.tensor(his).unsqueeze(0)
        assert his.shape == (1, T_in, d_Nin)

        #print('his', his)

        # Generate BirdView
        bv_img = np.zeros((224, 224, 3), np.float32)

        for other in other_trajs:
            assert len(other.state()) == T_in, len(other.state())

            for t in range(T_in):
                other_t = other.state()[t]
                x, y = self.coord_convert(other_t.x - start_point[0], other_t.y - start_point[1])

                if t + 1 == T_in:
                    cv2.circle(bv_img, (x, y), 1, (0.0, 1.0, 0.0), 5)
                else:
                    cv2.circle(bv_img, (x, y), 1, (0.0, 0.5 + 0.5 / T_in * t, 0.0), 3)
        
        for t in range(T_in):
            agent_t = my_traj.state()[t]
            x, y = self.coord_convert(agent_t.x - start_point[0], agent_t.y - start_point[1])

            if t + 1 == T_in:
                cv2.circle(bv_img, (x, y), 1, (0.0, 0.0, 1.0), 5)
            else:
                cv2.circle(bv_img, (x, y), 1, (0.0, 0.0, 0.5 + 0.5 / T_in * t), 3)

        bv_img = cv2.resize(bv_img, (64, 64))

        #cv2.imshow('bv_img', bv_img)
        #cv2.waitKey(0)
        
        bv_img = np.transpose(bv_img, (2, 0, 1))
        bv_img = torch.tensor(bv_img).unsqueeze(0)
        assert bv_img.shape == (1, 3, 64, 64)

        ##########################################

        his = his.to(self.dev)
        bv_img = bv_img.to(self.dev)

        preds = self.model((his, self.resnet(bv_img)))
        trajs = SampleTraj(preds, self.args['n_samples'])
        assert trajs.shape == (self.args['n_samples'], T_out, d_Nout)

        his = his.cpu().numpy()
        assert his.shape == (1, T_in, d_Nin), his.shape

        trajs = trajs + his[0, -1, :2] + start_point
        trajs = trajs.squeeze(0)
        assert trajs.shape == (T_out, d_Nout)

        self.results = trajs.copy()

    def fetch_my_state(self) -> MyState:
        assert self.last_state is not None
        assert self.results is not None

        ##################################################

        self._logger.info(f'predictor: Echo results back to simulator\n')

        traj = Trajectory()
        for i in range(30):
            s = State()

            s.track_id = self.last_state.track_id
            s.frame_id = self.last_state.frame_id + i + 1
            s.timestamp_ms = s.frame_id * 100
            s.agent_type = self.last_state.agent_type
            s.x = self.results[i][0]
            s.y = self.results[i][1]
            s.vx = ((self.results[i][0] - self.results[i - 1][0]) if i > 0 else self.results[i][0] - self.last_state.x) * 10
            s.vy = ((self.results[i][1] - self.results[i - 1][1]) if i > 0 else self.results[i][1] - self.last_state.y) * 10
            s.psi_rad = math.atan2(s.vy, s.vx)
            s.length = self.last_state.length
            s.width = self.last_state.width

            #print(s.x, s.y, s.vx, s.vy)

            traj.append_state(s)

        self.last_state = None
        self.results = None

        assert len(traj.state()) == 30

        res = MyState([traj], [1.0])
        return res

