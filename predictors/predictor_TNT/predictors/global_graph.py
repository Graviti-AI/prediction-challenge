import copy
import torch
from torch import nn
import torch.nn.functional as F
import math

def clones(module, N):
    return nn.ModuleList([copy.deepcopy(module) for _ in range(N)])

class Attention(nn.Module):
    r"""
    Self-Attention module, corresponding the global graph.
    Given lots of polyline vectors, each length is 'C', we want to get the predicted feature vector.
    """

    def __init__(self, C):
        r"""
        self.linear is 3 linear transformers for Q, K, V.
        :param C: the length of input feature vector. equals to the output dim of subgraph
        """
        super(Attention, self).__init__()
        self.linear = clones(nn.Linear(C, C), 3)
        self.dropout = nn.Dropout(p=0.1)
        # self.device = device
    def forward(self, P):
        r"""

        :param P: a list of polyline vectors, form a tensor.
                P.shape = [batch size, n, C]
        :param id: index of predicted vector.
                id.shape = [batch size]
        :return: output.
        """

        batchSize, n, C = P.shape
        # print(P.shape)

        # Q = torch.zeros(0, C).to('cuda:0')

        # Qt = self.linear[0](P)  # [batch size, n, C]
        # Q = Qt[:,0,:].unsqueeze(1)
        # print(q.shape)
        # for i in range(batchSize):
        #     # x = id[i].item()
        #     q = Qt[i, 0].unsqueeze(0)  # [1, C] the first is always the agent, note that map adds after all participants on the road
        #     Q = torch.cat((Q, q), dim=0)
        # Q.unsqueeze_(1)  # Q's shape is # [batch size, 1, C]

        # stupid Q
        # Q ,_= torch.max(self.linear[0](P), dim=1) #!!!
        # Q = Q.unsqueeze(1)
        Q = self.linear[0](P)
        K = self.linear[1](P)  # [batch size, n, C]
        V = self.linear[2](P)

        Adj = torch.matmul(Q, K.permute(0, 2, 1)) / math.sqrt(C)  # [batch size, n, n]
        Adj = F.softmax(Adj, dim=2)
        gcn_layer = torch.matmul(Adj, V)  # [batch size, n, C]
        agent_feature = gcn_layer[:,0,:]

        return agent_feature
