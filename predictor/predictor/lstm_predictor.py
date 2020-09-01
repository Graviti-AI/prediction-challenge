# By SYF, 8/20/2020

import torch
import torch.nn as nn
import torch.nn.functional as F

import os
import logging
import numpy as np

from predictor.traj import *
from predictor.predictor import Predictor, MyState


class LSTMPredictor(Predictor):

    def __init__(self, logger: logging.Logger):
        super().__init__()

        self._logger = logger
        self.args = {
            'n_gaussians': 5,
            'n_samples': 1,
            'encoder': 'gru',
            'decoder': 'mdn',
            'K': 2,
            'd_h': 256,
            'load': 'lstm.pt',
        }
        print('args', self.args)

        self.dev = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        print('dev', self.dev)

        ##########################################
        self.model = MODEL1(
            self.args['encoder'], self.args['decoder'], self.args['K'],
            self.args['d_h'], self.args['n_gaussians'], self.dev
        )
        self.model.to(self.dev)

        self.model.load_state_dict(torch.load(os.path.join(os.path.dirname(__file__), 'lstm.pt')))
        self.model.eval()

        ##########################################

        self.last_state = None
        self.results = None

    def start(self):
        pass

    def shutdown(self):
        pass

    def on_env(self, map_name, my_traj: Trajectory, other_trajs: []):
        # assert self.last_state is None #TODO:
        # assert self.results is None

        #############################################

        self._logger.info(f'predictor: Receive from Simulator')
        assert len(my_traj.state()) == 10, 'The length of the historical trajectory must be 10'

        his = []
        for state in my_traj.state():
            self._logger.info(f'frame_id: {state.frame_id}; x: {state.x}; y: {state.y}')
            his.append([state.x, state.y])

            self.last_state = state

        his = np.array(his).astype(np.float32)
        assert his.shape == (10, 2)

        start_point = his[0].copy()
        assert start_point.shape == (2,)

        inputs = his - start_point
        inputs = torch.tensor(inputs).unsqueeze(0)
        assert inputs.shape == (1, 10, 2)

        ##########################################

        preds = self.model(inputs)
        trajs = SampleTraj(preds, self.args['n_samples'])
        assert trajs.shape == (1, 30, 2)

        inputs = inputs.numpy()
        trajs = trajs + inputs[:, -1, :] + start_point
        trajs = trajs.squeeze(0)
        assert trajs.shape == (30, 2)

        #########################################
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
            s.vx = (self.results[i][0] - self.results[i - 1][0]) if i > 0 else self.results[i][0] - self.last_state.x
            s.vy = (self.results[i][1] - self.results[i - 1][1]) if i > 0 else self.results[i][1] - self.last_state.y
            s.psi_rad = 0.0  # TODO:
            s.length = self.last_state.length
            s.width = self.last_state.width
            traj.append_state(s)

        self.last_state = None
        self.results = None

        assert len(traj.state()) == 30

        res = MyState([traj], [1.0])
        return res


############################
#       LSTM + MDN
############################

T_in = 10
T_out = 30
T_tot = 40

d_Nin = 2
d_Nout = 2


class MLP(nn.Module):
    """MLP Block : Linear + relu"""

    def __init__(self, d_in, d_out):
        super().__init__()

        self.d_out = d_out

        self.linear1 = nn.Linear(d_in, d_out)
        self.relu1 = nn.ReLU()

    def forward(self, x):
        x = self.relu1(self.linear1(x))
        return x


class GRU_encoder(nn.Module):
    """GRU Encoder Module"""

    def __init__(self, d_in, d_h):
        super().__init__()

        self.d_h = d_h
        self.gru = nn.GRU(d_in, d_h)

    def forward(self, x):
        """
        Input:  (B, T_in, d_Nin)
        Output: (B, d_h)
        """

        self.gru.flatten_parameters()

        B = x.shape[0]
        x = x.permute(1, 0, 2)

        _, last_hid = self.gru(x)
        assert last_hid.shape == (1, B, self.d_h), last_hid.shape

        last_hid = last_hid.reshape(B, self.d_h)
        return last_hid


class MDN_decoder(nn.Module):
    """Mixture Density Networks"""

    def __init__(self, d_h, n_gaussians):
        super().__init__()

        self.n_gaussians = n_gaussians
        self.pi = nn.Linear(d_h, T_out * d_Nout * n_gaussians)
        self.mu = nn.Linear(d_h, T_out * d_Nout * n_gaussians)
        self.sigma = nn.Linear(d_h, T_out * d_Nout * n_gaussians)

    def forward(self, x):
        """
        Input:  (B, d_h)
        Output: pi, mu, sigma
        """

        B = x.shape[0]

        pi = self.pi(x)
        sigma = self.sigma(x)
        mu = self.mu(x)

        pi = pi.reshape(B, T_out * d_Nout, self.n_gaussians)
        pi = F.softmax(pi, dim=-1)

        sigma = - F.relu(sigma)
        sigma = torch.exp(sigma)
        sigma = sigma.reshape(B, T_out * d_Nout, self.n_gaussians)

        mu = mu.reshape(B, T_out * d_Nout, self.n_gaussians)

        return pi, mu, sigma


class MODEL1(nn.Module):

    def __init__(self, encoder, decoder, K, d_h, n_gaussians, dev):
        super().__init__()

        self.K = K
        self.dev = dev

        self.encoder_agent = GRU_encoder(d_Nin, d_h)
        self.FCs = nn.ModuleList(
            [MLP(d_h, d_h) for _ in range(K)]
        )

        if decoder == 'mdn':
            self.decoder = MDN_decoder(d_h, n_gaussians)
        else:
            assert False

    def forward(self, inputs):
        """
        Arguments:
            agent -- (B, T_in, d_Nin)
            bv_img -- (B, 1000)

        Returns:
            preds -- (B, T_out, d_Nout)
        """

        agent = inputs.to(self.dev)
        x = self.encoder_agent(agent)

        for i in range(self.K):
            x = self.FCs[i](x)

        pi, mu, sigma = self.decoder(x)
        return pi, mu, sigma


def gumbel_sample(x, axis):
    z = np.random.gumbel(loc=0, scale=1, size=x.shape)
    return (np.log(x + 1e-20) + z).argmax(axis=axis)


def SampleTraj(preds, n_samples):
    """
    pi, mu, sigma (B, T_out * d_Nout, self.n_gaussians)
    """

    pi, mu, sigma = preds
    assert len(pi) == 1 and len(mu) == 1 and len(sigma) == 1

    pi = pi.detach().cpu().numpy()[0]
    mu = mu.detach().cpu().numpy()[0]
    sigma = sigma.detach().cpu().numpy()[0]

    k = gumbel_sample(np.tile(pi, (n_samples, 1, 1)), -1)
    assert k.shape == (n_samples, T_out * d_Nout), k.shape

    sampled = [
        [
            mu[t][k[x][t]]
            + sigma[t][k[x][t]] * np.random.rand()

            for t in range(T_out * d_Nout)
        ]
        for x in range(n_samples)
    ]

    sampled = np.array(sampled).astype(np.float32)
    assert sampled.shape == (n_samples, T_out * d_Nout), sampled.shape

    sampled = sampled.reshape(n_samples, T_out, d_Nout)
    sampled = np.cumsum(sampled, axis=1)

    return sampled
