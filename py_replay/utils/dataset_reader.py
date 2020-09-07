#!/usr/bin/env python

import csv
import math
from utils.dataset_types import MotionState, Track, Measurements


class Config:

    def __init__(self):
        super().__init__()
        self.map = None

def read_config(filename) -> Config:
    config = Config()

    with open(filename, 'r') as fin:
        line  = fin.readline().strip(' \n')
        assert line[:4] == 'Map:'

        config.map = line[4:]

    return config




def read_log(filename) -> {}:
    track_dict = dict()

    with open(filename) as fin:
        fin.readline()
        fin.readline()
        fin.readline()

        flag = True
        while flag:
            frame_id = int(fin.readline())

            while True:
                line = fin.readline().strip()

                if not line:
                    flag = False
                    break

                if line == '-----------------':
                    break
                
                info = list(line.strip().split(','))
                assert len(info) == 9, info

                # extract info
                track_id = int(info[0])
                time_stamp_ms = frame_id * 100
                agent_type = 'car'
                x = float(info[1])
                y = float(info[2])
                vx = float(info[4])
                vy = float(info[5])
                psi_rad = float(info[3])
                length = float(info[7])
                width = float(info[8])

                if not track_id in track_dict:
                    track = Track(track_id)
                    track.agent_type = agent_type
                    track.length = length
                    track.width = width
                    track.time_stamp_ms_first = time_stamp_ms
                    track.time_stamp_ms_last = time_stamp_ms
                    track_dict[track_id] = track
                
                track = track_dict[track_id]
                track.time_stamp_ms_last = time_stamp_ms

                # MotionState
                ms = MotionState(time_stamp_ms)
                ms.x = x
                ms.y = y
                ms.vx = vx
                ms.vy = vy
                ms.psi_rad = psi_rad
                track.motion_states[ms.time_stamp_ms] = ms

                # Measurements
                me = Measurements(time_stamp_ms)
                me.velo = math.sqrt(ms.vx ** 2 + ms.vy ** 2)

                if not ((time_stamp_ms - 100) in track.measurements):
                    me.jerk = 0.0
                    me.delta_yaw = 0.0
                else:
                    me.jerk = me.velo - track.measurements[time_stamp_ms - 100].velo
                    me.delta_yaw = ms.psi_rad - track.motion_states[time_stamp_ms - 100].psi_rad
                
                track.measurements[time_stamp_ms] = me
    
    return track_dict



'''
class Key:
    track_id = "track_id"
    frame_id = "frame_id"
    time_stamp_ms = "timestamp_ms"
    agent_type = "agent_type"
    x = "x"
    y = "y"
    vx = "vx"
    vy = "vy"
    psi_rad = "psi_rad"
    length = "length"
    width = "width"


class KeyEnum:
    track_id = 0
    frame_id = 1
    time_stamp_ms = 2
    agent_type = 3
    x = 4
    y = 5
    vx = 6
    vy = 7
    psi_rad = 8
    length = 9
    width = 10


def read_tracks(filename):

    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')

        track_dict = dict()
        track_id = None

        for i, row in enumerate(list(csv_reader)):

            if i == 0:
                # check first line with key names
                assert(row[KeyEnum.track_id] == Key.track_id)
                assert(row[KeyEnum.frame_id] == Key.frame_id)
                assert(row[KeyEnum.time_stamp_ms] == Key.time_stamp_ms)
                assert(row[KeyEnum.agent_type] == Key.agent_type)
                assert(row[KeyEnum.x] == Key.x)
                assert(row[KeyEnum.y] == Key.y)
                assert(row[KeyEnum.vx] == Key.vx)
                assert(row[KeyEnum.vy] == Key.vy)
                assert(row[KeyEnum.psi_rad] == Key.psi_rad)
                assert(row[KeyEnum.length] == Key.length)
                assert(row[KeyEnum.width] == Key.width)
                continue

            if int(row[KeyEnum.track_id]) != track_id:
                # new track
                track_id = int(row[KeyEnum.track_id])
                assert(track_id not in track_dict.keys()), \
                    "Line %i: Track id %i already in dict, track file not sorted properly" % (i+1, track_id)
                track = Track(track_id)
                track.agent_type = row[KeyEnum.agent_type]
                track.length = float(row[KeyEnum.length])
                track.width = float(row[KeyEnum.width])
                track.time_stamp_ms_first = int(row[KeyEnum.time_stamp_ms])
                track.time_stamp_ms_last = int(row[KeyEnum.time_stamp_ms])
                track_dict[track_id] = track

            track = track_dict[track_id]
            track.time_stamp_ms_last = int(row[KeyEnum.time_stamp_ms])
            ms = MotionState(int(row[KeyEnum.time_stamp_ms]))
            ms.x = float(row[KeyEnum.x])
            ms.y = float(row[KeyEnum.y])
            ms.vx = float(row[KeyEnum.vx])
            ms.vy = float(row[KeyEnum.vy])
            ms.psi_rad = float(row[KeyEnum.psi_rad])
            track.motion_states[ms.time_stamp_ms] = ms

        return track_dict


def read_pedestrian(filename):

    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')

        track_dict = dict()
        track_id = None

        for i, row in enumerate(list(csv_reader)):

            if i == 0:
                # check first line with key names
                assert (row[KeyEnum.track_id] == Key.track_id)
                assert (row[KeyEnum.frame_id] == Key.frame_id)
                assert (row[KeyEnum.time_stamp_ms] == Key.time_stamp_ms)
                assert (row[KeyEnum.agent_type] == Key.agent_type)
                assert (row[KeyEnum.x] == Key.x)
                assert (row[KeyEnum.y] == Key.y)
                assert (row[KeyEnum.vx] == Key.vx)
                assert (row[KeyEnum.vy] == Key.vy)
                continue

            if row[KeyEnum.track_id] != track_id:
                # new track
                track_id = row[KeyEnum.track_id]
                assert (track_id not in track_dict.keys()), \
                    "Line %i: Track id %s already in dict, track file not sorted properly" % (i + 1, track_id)
                track = Track(track_id)
                track.agent_type = row[KeyEnum.agent_type]
                track.time_stamp_ms_first = int(row[KeyEnum.time_stamp_ms])
                track.time_stamp_ms_last = int(row[KeyEnum.time_stamp_ms])
                track_dict[track_id] = track

            track = track_dict[track_id]
            track.time_stamp_ms_last = int(row[KeyEnum.time_stamp_ms])
            ms = MotionState(int(row[KeyEnum.time_stamp_ms]))
            ms.x = float(row[KeyEnum.x])
            ms.y = float(row[KeyEnum.y])
            ms.vx = float(row[KeyEnum.vx])
            ms.vy = float(row[KeyEnum.vy])
            track.motion_states[ms.time_stamp_ms] = ms

        return track_dict
'''