import torch
import torch.nn as nn
import torch.nn.functional as F

from predictors.config import *

import numpy as np

########################
#      Basical Layer
########################


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



########################
#      Encoder
########################


class MLP_encoder(nn.Module):

    def __init__(self, d_h):
        super().__init__()

        self.mlp1 = MLP(T_in * d_Nin, d_h)
    
    def forward(self, x):
        """
        Input:  (B, T_in, d_Nin)
        Output: (B, d_h)
        """

        x = x.reshape(x.shape[0], T_in * d_Nin)
        return self.mlp1(x)


class GRU_encoder(nn.Module):
    """GRU Encoder Module"""

    def __init__(self, d_in, d_h):
        super().__init__()

        self.d_h = d_h
        #self.embed = MLP(d_Nin * (N + 1), d_h)
        #self.embed = MLP(d_in, d_h)
        self.gru = nn.GRU(d_in, d_h)

    def forward(self, x):
        """
        Input:  (B, T_in, d_Nin)
        Output: (B, d_h)
        """

        #x = self.embed(x)
        self.gru.flatten_parameters()

        B = x.shape[0]
        x = x.permute(1, 0, 2)

        _, last_hid = self.gru(x)
        assert last_hid.shape == (1, B, self.d_h), last_hid.shape

        last_hid = last_hid.reshape(B, self.d_h)
        return last_hid



########################
#      Decoder
########################


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

        pi = self.pi(x).reshape(B, T_out * d_Nout, self.n_gaussians)
        pi = F.softmax(pi, dim=-1)

        sigma = - F.relu(self.sigma(x)) - 1
        sigma = torch.exp(sigma)
        sigma = sigma + 0.0001
        sigma = sigma.reshape(B, T_out * d_Nout, self.n_gaussians)

        mu = self.mu(x)
        mu = mu.reshape(B, T_out * d_Nout, self.n_gaussians)

        return pi, mu, sigma



########################
#      ModelX
########################


class ModelX(nn.Module):

    def __init__(self, encoder, decoder, K, d_h, n_gaussians, dev):
        super().__init__()

        self.K = K
        self.dev = dev

        self.encoder_agent = GRU_encoder(d_Nin, d_h)
        self.bv_img_fc = MLP(1000, d_h)

        self.fusion = MLP(2 * d_h, d_h)

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

        agent, bv_img = inputs
        
        x0 = self.encoder_agent(agent)
        x1 = self.bv_img_fc(bv_img)

        x = torch.cat([x0, x1], axis = -1)
        x = self.fusion(x)

        for i in range(self.K):
            x = self.FCs[i](x)

        pi, mu, sigma = self.decoder(x)

        #print('sigma', sigma[0][0].max().item(), sigma[0][0].min().item(), sigma[0][0].mean().item())

        return pi, mu, sigma



########################
#      Sample
########################

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

    assert n_samples == 1
    sampled = [
        [
            mu[t][k[x][t]]
            #+ sigma[t][k[x][t]] * np.random.rand()

            for t in range(T_out * d_Nout)
        ]
        for x in range(n_samples)
    ]

    sampled = np.array(sampled).astype(np.float32)
    assert sampled.shape == (n_samples, T_out * d_Nout), sampled.shape

    sampled = sampled.reshape(n_samples, T_out, d_Nout)
    sampled = np.cumsum(sampled, axis=1)

    return sampled