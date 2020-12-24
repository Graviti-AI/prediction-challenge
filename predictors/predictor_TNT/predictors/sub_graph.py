import torch
from torch import nn
import torch.nn.functional as F

class SubGraph(nn.Module):
    r"""
      Subgraph of VectorNet. This network accept a number of initiated vectors belong to
    the same polyline, flow three layers of network, then output this polyline's feature vector.
    """

    def __init__(self, feature_length, layersNumber):
        r"""
          Given all vectors of this polyline, we should build a 3-layers subgraph network,
        get the output which is the polyline's feature vector.
        :param feature_length: the length of vector.
        :param layersNumber: the number of subgraph network.
        """
        super(SubGraph, self).__init__()
        self.layers_number = layersNumber
        self.layers = nn.ModuleList([SubGraphLayer(feature_length * (2 ** i)) for i in range(self.layers_number)])

    def forward(self, x):
        r"""

        :param x: a number of vectors. x.shape=[batch size, vNumber, feature_length].
        :return: The vector of this polyline. Shape is [batch size, output feature_length].
        """

        # x = torch.tensor(
        #     [[[1, 0, 0, 0, 0, 0, 0, 0, 0],
        #       [1, 2, 3, 1, -1, -2, -3, -1, 1],
        #       [3, 2, 1, 2, 3, 1, -1, -2, -3]],
        #
        #      [[0, 0, 0, 0, 2, 1, 2, 3, 1],
        #       [1, 3, 2, 1, 3, 1, -1, -2, -3],
        #       [3, 3, 3, 2, 0, 0, 0, 2, 1]]]).float().to(device)
        # print(x.shape)
        for layer in self.layers:
            # print('sub graph !!!')
            x = layer(x)
        # x's shape is [batch size, vNumber, output feature_length]


        x = x.permute(0, 2, 1)  # [batch size, output feature_length, vNumber]
        x = F.max_pool1d(x, kernel_size=x.shape[2])  # [batch size, output feature_length, 1]
        x = x.permute(0, 2, 1)  # [batch size, 1, output feature_length]
        x.squeeze_(1)

        return x


class SubGraphLayer(nn.Module):
    r"""
      One layer of subgraph, include the MLP of g_enc.
      The calculation detail in this paper's 3.2 section.
      Input some vectors with 'feature_length' length, the output's length is '2*feature_length'(because of
    concat operator).
    """

    def __init__(self, feature_length):
        r"""

        :param feature_length: the length of input vector.
        """
        super(SubGraphLayer, self).__init__()
        self.mlp_input_size = feature_length
        self.mlp_output_size = feature_length
        self.hidden_size = 64
        self.node_encoder = nn.Sequential(nn.Linear(self.mlp_input_size, self.mlp_output_size),
                                    nn.LayerNorm(self.mlp_output_size),
                                    nn.ReLU(True),
                                    # nn.Linear(self.hidden_size, self.mlp_output_size),
                                    # nn.ReLU(True)
                                    )

    def forward(self, x):
        r"""

        :param x: A number of vectors. x.shape = [batch size, vNumber, feature_length]
        :return: All processed vectors with shape [batch size, vNumber, feature_length*2]
        """
        # print(x.shape)
        x = self.node_encoder(x.double())
        batchSize, vNumber, feature_length = x.shape

        x = x.permute(1, 0, 2)  # [vNumber, batch size, feature_length]

        mp = x.permute(1, 2, 0) # [batch size, feature_length, vNumber]
        mp = F.max_pool1d(mp, kernel_size=mp.shape[2])  # [batch size, feature_length, 1]

        mp = torch.cat([mp] * vNumber, dim=2)  # [batch size, feature_length, vNumber]
 
        y = torch.cat((mp.permute(0, 2, 1), x.permute(1, 0, 2)), dim=2) # expand dim: feature_length*2
        
        return y
