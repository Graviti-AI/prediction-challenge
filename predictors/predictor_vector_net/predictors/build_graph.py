import os
import sys
import csv
from predictors.utils.dataset_types import MotionState, Track
from predictors.utils.dataset_reader import Key
import numpy as np
import math
from predictors.utils import segmentation

import networkx as nx
import matplotlib.pyplot as plt

class Key_seg(Key):
    agent_role = 'agent_role'
    case_id = 'case_id'

class dataloader(object):
    def __init__(self, data_path):
        self.time_stamp = 0
        self.data_path = data_path
        # self.nodes = []
        self.featuremap = []
        # self.positions = []
        self.track_dict = dict()
    def build_graph(self, time_stamp):
        self.time_stamp = time_stamp
        self.title, self.data =  segmentation.load_csv(self.data_path)
        self.get_title_pos(self.title)
        self.track_dict = self.read_specific_tracks(self.data, self.time_stamp)
        self.calc_dis_metric()
        self.dis_metric[self.dis_metric>30] = 0
        self.visualize()
    def cal_dis(self, pos1, pos2):
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
    def get_title_pos(self,title):
        self.time_pos = title.index(Key_seg.time_stamp_ms)
        self.track_pos = title.index(Key_seg.track_id)
        self.agent_type_pos = title.index(Key_seg.agent_type)
        self.agent_role_pos = title.index(Key_seg.agent_role)
        self.len_pos = title.index(Key_seg.length)
        self.wid_pos = title.index(Key_seg.width)
        self.x_pos = title.index(Key_seg.x)
        self.y_pos = title.index(Key_seg.y)
        self.vx_pos = title.index(Key_seg.vx)
        self.vy_pos = title.index(Key_seg.vy)
        self.psi_pos = title.index(Key_seg.psi_rad)
    def read_specific_tracks(self, data, time_stamp):
        track_dict =dict()
        track_id = None
        for i, row in enumerate(data):
            if int(row[self.time_pos]) > time_stamp:
                break
            if int(row[self.track_pos]) != track_id:
                # new track
                track_id = row[self.track_pos]
                assert(track_id not in track_dict.keys()), \
                    "Line %i: Track id %i already in dict, track file not sorted properly" % (i+1, track_id)
                track = Track(track_id)
                track.agent_type = row[self.agent_type_pos]
                track.length = float(row[self.len_pos])
                track.width = float(row[self.wid_pos])
                track.time_stamp_ms_first = int(row[self.time_pos])
                track.time_stamp_ms_last = int(row[self.time_pos])
                track_dict[track_id] = track

            track = track_dict[track_id]
            track.time_stamp_ms_last = int(row[self.time_pos])
            ms = MotionState(int(row[self.time_pos]))
            ms.x = float(row[self.x_pos])
            ms.y = float(row[self.y_pos])
            ms.vx = float(row[self.vx_pos])
            ms.vy = float(row[self.vy_pos])
            ms.psi_rad = float(row[self.psi_pos])
            track.motion_states[ms.time_stamp_ms] = ms
        return track_dict
    def calc_dis_metric(self):
        self.num_of_nodes = len(self.track_dict)
        self.dis_metric = np.zeros((self.num_of_nodes, self.num_of_nodes))
        key_list = []
        for key in self.track_dict:
            key_list.append(key)
        assert (len(key_list) == self.num_of_nodes)

        for i in range(self.num_of_nodes):
            for j in range(i+1,self.num_of_nodes): 
                pos1 = []
                pos2 = []
                pos1.append(self.track_dict[key_list[i]].motion_states[self.time_stamp].x);pos1.append(self.track_dict[key_list[i]].motion_states[self.time_stamp].y)
                pos2.append(self.track_dict[key_list[j]].motion_states[self.time_stamp].x);pos2.append(self.track_dict[key_list[j]].motion_states[self.time_stamp].y)
                self.dis_metric[i][j] = self.cal_dis(pos1,pos2)
        self.dis_metric += self.dis_metric.T
            
    def visualize(self):
        G = nx.Graph()
        edge_list = []
        for i in range(self.num_of_nodes):
            for j in range(i+1,self.num_of_nodes):
                if self.dis_metric[i][j] != 0:
                    edge_list.append((i,j,self.dis_metric[i][j]))
        G.add_weighted_edges_from(edge_list)
        nx.draw(G,with_labels=True, node_color='b')
        plt.show()

    
'''
if __name__ == '__main__':
    csv_path = '/home/jonathon/Documents/new_project/interaction-dataset-master/recorded_trackfiles/DR_CHN_Merging_ZS/train/segmented/tracks_000.csv'
    graph1 = dataloader(csv_path)
    graph1.build_graph(100)
    print(graph1.dis_metric.shape)
'''
