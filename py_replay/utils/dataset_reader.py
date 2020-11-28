import re
import csv
import math
import numpy as np


DELTA_TIMESTAMP_MS = 100     # each tick in the simulator is 0.1s



class Config:

    def __init__(self, filename, verbose=False):
        self.map = None
        self.csv_id = None
        self.StartTimestamp = None
        self.EndTimestamp = None
        self.robot_car_planner = None

        self.EgoEndPositionX = None
        self.EgoEndPositionY = None
        self.TargetRightofWay = None

        self.read_from_file(filename)
        if verbose:
            self.print()
    
    def print(self):
        print("########## config info ##########")
        print("# map: ", self.map)
        print("# StartTimestamp: ", self.StartTimestamp)
        print("# EndTimestamp: ", self.EndTimestamp)
        print("# EgoEndPositionX: ", self.EgoEndPositionX)
        print("# EgoEndPositionY: ", self.EgoEndPositionY)
        print("# TargetRightofWay: ", self.TargetRightofWay)
        print()

    def read_from_file(self, filename):
        with open(filename, 'r') as fin:
            line  = fin.readline().strip()
            assert line[:4] == 'Map:'
            self.map = line[4:]

            line = fin.readline().strip()
            assert line[:6] == 'Track:'
            self.csv_id = line[6:]

            line = fin.readline().strip()
            assert line[:19] == 'StartTimestamp(ms):'
            self.StartTimestamp = int(line[19:])

            line = fin.readline().strip()
            assert line[:17] == 'EndTimestamp(ms):'
            self.EndTimestamp = int(line[17:])

            line = fin.readline().strip()
            assert line[:12] == 'RobotCarNum:'
            robot_car_num = int(line[12:])

            line = fin.readline().strip()
            assert line == 'InitState:track_id,Planner,Planner.Para,in_Predictor,in_Predictor.dt,in_Predictor.horizon,ex_Predictor,ex_Predictor.dt,ex_Predictor.horizon,ego_car'

            self.robot_car_planner = {}
            for _ in range(robot_car_num):
                line = fin.readline().strip()
                info = line.split(' ')
                self.robot_car_planner[int(info[0])] = info[1]
            
            assert len(self.robot_car_planner) == robot_car_num

            while line[:15] != 'EgoEndPosition:':
                line = fin.readline().strip()
            
            info = list(line[15:].strip().split(' '))
            self.EgoEndPositionX = float(info[0])
            self.EgoEndPositionY = float(info[1])

            line = fin.readline().strip()
            assert line[:17] == 'TargetRightofWay:'

            if len(line) > 17:
                self.TargetRightofWay = [int(x) for x in list(line[17:].strip().split(' '))]
            else:
                self.TargetRightofWay = []



class Collision:

    def __init__(self, filename, verbose=False):
        self.record_with_car = {}
        self.record_with_lane = {}
        self.read_from_file(filename)

        if verbose:
            self.print()
    
    def print(self):
        print("########## collision info ##########")
        print("# record_with_car ts:", len(self.record_with_car))
        print("# record_with_car number:", sum([len(self.record_with_car[ts]) for ts in self.record_with_car]))
        print("# record_with_lane ts:", len(self.record_with_lane))
        print("# record_with_lane number:", sum([len(self.record_with_lane[ts]) for ts in self.record_with_lane]), '\n')

    def add_item(self, record, ts, car, other):
        if not ts in record:
            record[ts] = []

        record[ts].append((car, other))

    def read_from_file(self, filename):
        with open(filename) as fin:
            fin.readline()
            fin.readline()

            while True:
                line = fin.readline().strip()
                if not line:
                    break

                info = list(re.split('[ :]', line))
                ts = int(info[0]) * DELTA_TIMESTAMP_MS
                car = int(info[-2])
                other = int(info[-1])

                c_type = info[-3]
                assert c_type in ['CarID)', 'laneID)'], c_type

                if c_type == 'CarID)':
                    self.add_item(self.record_with_car, ts, car, other)
                else:
                    self.add_item(self.record_with_lane, ts, car, other)



class MotionState:

    def __init__(self, time_stamp_ms):
        assert isinstance(time_stamp_ms, int)

        self.time_stamp_ms = time_stamp_ms
        self.x = None
        self.y = None
        self.vx = None
        self.vy = None
        self.psi_rad = None
        self.lane_id = None
        self.centerline = None

        self.jerk = None
        self.velo = None
        self.yaw2lane = None

        self.in_pred = None
        self.ex_pred = None
        self.pred_loss = None

    def __str__(self):
        res = 'x: %.2lf, y: %.2lf, vx: %.2lf, vy: %.2lf, psi_rad: %.2lf, lane_id: %d, centerline: %.2lf' % (self.x, self.y, self.vx, self.vy, self.psi_rad, self.lane_id, self.centerline)
        res += ' | jerk: %.2lf, velo: %.2lf, yaw2lane: %.2lf' % (self.jerk, self.velo, self.yaw2lane)

        return res



class Track:

    def __init__(self, track_id):
        assert isinstance(track_id, int)

        self.track_id = track_id
        self.agent_type = None
        self.length = None
        self.width = None
        self.time_stamp_ms_first = None
        self.time_stamp_ms_last = None
        self.isego = None
        self.s_now = None
        self.s_tot = None
        self.motion_states = dict()
    
    def print(self):
        print("# ID (%d), %s, ts [%d, %d], is_ego: %s" % (self.track_id, self.agent_type, self.time_stamp_ms_first, self.time_stamp_ms_last, self.isego))



class Log:

    def __init__(self, filename, verbose=False):
        self.track_dict = dict()
        self.no_crash = False

        self.read_from_file(filename)
        if verbose:
            self.print()
    
    def print(self):
        print("########## log info ##########")
        for key in self.track_dict:
            self.track_dict[key].print()
        
        print("# no_crash", self.no_crash, '\n')
    
    def read_from_file(self, filename):
        with open(filename) as fin:
            fin.readline()
            fin.readline()
            fin.readline()

            while True:
                header = fin.readline()

                if not header:
                    break       # end of file

                header = header.strip()
                if header == "no crash":
                    self.no_crash = True

                    # read final state
                    line = fin.readline().strip()
                    assert line == "EgoCarFinalState:id,s_now,s_tot"

                    line = fin.readline()
                    if not line:
                        for track_id in self.track_dict:
                            if self.track_dict[track_id].isego == 'yes':
                                self.track_dict[track_id].s_now = 10000000.0
                                self.track_dict[track_id].s_tot = 10000000.0
                                print('WARNING: can not find ego car!')
                        break

                    info = list(line.strip().split(','))
                    assert len(info) == 3, info

                    t_id = int(info[0])
                    s_now = float(info[1])
                    s_tot = float(info[2])

                    assert t_id in self.track_dict, t_id
                    assert self.track_dict[t_id].isego == 'yes', self.track_dict[t_id]

                    self.track_dict[t_id].s_now = s_now
                    self.track_dict[t_id].s_tot = s_tot

                    break

                assert header == '-----------------', header
                frame_id, car_number = map(int, list(fin.readline().strip().split(' ')))

                for _ in range(car_number):
                    line = fin.readline().strip()

                    if not line:
                        assert False, "incompleted log file"
                    
                    info = list(line.strip().split(','))
                    assert len(info) == 13, info
                    
                    track_id = int(info[0])
                    time_stamp_ms = frame_id * DELTA_TIMESTAMP_MS
                    x = float(info[1])
                    y = float(info[2])
                    vx = float(info[4])
                    vy = float(info[5])
                    psi_rad = float(info[3])
                    length = float(info[7])
                    width = float(info[8])
                    lane_id = int(info[9])
                    centerline = float(info[10])
                    agent_type = info[11]
                    isego = info[12]        # yes / no

                    assert agent_type in ['BehaveCar', 'ReplayCar'], agent_type
                    assert isego in ['yes', 'no'], isego

                    if not track_id in self.track_dict:
                        track = Track(track_id)
                        track.length = length
                        track.width = width
                        track.time_stamp_ms_first = time_stamp_ms
                        track.time_stamp_ms_last = time_stamp_ms
                        track.agent_type = agent_type
                        track.isego = isego
                        self.track_dict[track_id] = track
                    
                    track = self.track_dict[track_id]
                    track.time_stamp_ms_last = time_stamp_ms

                    # MotionState
                    ms = MotionState(time_stamp_ms)
                    ms.x = x
                    ms.y = y
                    ms.vx = vx
                    ms.vy = vy
                    ms.psi_rad = psi_rad
                    ms.lane_id = lane_id
                    ms.centerline = centerline
                    ms.velo = math.sqrt(ms.vx ** 2 + ms.vy ** 2)
                    ms.yaw2lane = ms.psi_rad - ms.centerline

                    # calc jerk
                    if time_stamp_ms < 2 * DELTA_TIMESTAMP_MS + track.time_stamp_ms_first:
                        ms.jerk = 0.0
                    else:
                        a = (ms.velo - track.motion_states[time_stamp_ms - DELTA_TIMESTAMP_MS].velo) / (DELTA_TIMESTAMP_MS / 1000)
                        b = (track.motion_states[time_stamp_ms - DELTA_TIMESTAMP_MS].velo - \
                            track.motion_states[time_stamp_ms - DELTA_TIMESTAMP_MS * 2].velo) / (DELTA_TIMESTAMP_MS / 1000)
                        ms.jerk = (a - b) / (DELTA_TIMESTAMP_MS / 1000)
                    
                    #if isego == 'yes':
                    #    print('track_id: %d, time_stamp_ms: %d, x: %.3lf, y: %.3lf, vx: %.3lf, vy: %.3lf, jerk: %.3lf' % (track_id, time_stamp_ms, x, y, vx, vy, ms.jerk))
                    
                    track.motion_states[ms.time_stamp_ms] = ms
    
    def read_prediction(self, filename, verbose=False):
        with open(filename) as fin:
            while True:
                line = fin.readline()
                if not line:
                    break
                
                if line[:7] == "# DEBUG":
                    info = line.strip().split(' ')

                    assert info[3] == "UpdateTime:", info[3]
                    assert info[5] == 'id:', info[5]

                    timestep_ts = int(info[4]) * DELTA_TIMESTAMP_MS
                    t_id = int(info[6])

                    ###################################################

                    line = fin.readline().strip()
                    assert line == '# DEBUG | in_length: 31 ex_length: 31', line

                    ###################################################

                    in_pred = []
                    ex_pred = []
                    for _ in range(31):
                        line = fin.readline().strip().split(' ')
                        
                        assert line[1] == 'DEBUG', line[1]
                        assert line[3] == 't:', line[3]
                        assert line[6] == 'IN', line[6]
                        assert line[7] == 'x:', line[7]
                        assert line[9] == 'y:', line[9]
                        assert line[14] == 'EX', line[14]
                        assert line[15] == 'x:', line[15]
                        assert line[17] == 'y:', line[17]

                        in_pred.append([float(line[8]), float(line[10])])
                        ex_pred.append([float(line[16]), float(line[18])])
                    
                    in_pred = np.array(in_pred)
                    ex_pred = np.array(ex_pred)

                    ####################################################

                    assert t_id in self.track_dict, t_id
                    assert timestep_ts in self.track_dict[t_id].motion_states, (t_id, timestep_ts)

                    self.track_dict[t_id].motion_states[timestep_ts].in_pred = in_pred
                    self.track_dict[t_id].motion_states[timestep_ts].ex_pred = ex_pred

                    line = fin.readline().strip()
                    assert line[:8] == '# Loss: ', line[:8]

                    self.track_dict[t_id].motion_states[timestep_ts].pred_loss = float(line[8:])


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


class Dataset:

    def __init__(self, filename):
        self.track_dict = {}
        self.read_from_file(filename)
    
    def read_from_file(self, filename):
        
        with open(filename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
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
                    assert(track_id not in self.track_dict.keys()), \
                        "Line %i: Track id %i already in dict, track file not sorted properly" % (i+1, track_id)
                    track = Track(track_id)
                    track.agent_type = row[KeyEnum.agent_type]
                    track.length = float(row[KeyEnum.length])
                    track.width = float(row[KeyEnum.width])
                    track.time_stamp_ms_first = int(row[KeyEnum.time_stamp_ms])
                    track.time_stamp_ms_last = int(row[KeyEnum.time_stamp_ms])
                    self.track_dict[track_id] = track

                track = self.track_dict[track_id]
                track.time_stamp_ms_last = int(row[KeyEnum.time_stamp_ms])
                ms = MotionState(int(row[KeyEnum.time_stamp_ms]))
                ms.x = float(row[KeyEnum.x])
                ms.y = float(row[KeyEnum.y])
                ms.vx = float(row[KeyEnum.vx])
                ms.vy = float(row[KeyEnum.vy])
                ms.psi_rad = float(row[KeyEnum.psi_rad])
                track.motion_states[ms.time_stamp_ms] = ms
