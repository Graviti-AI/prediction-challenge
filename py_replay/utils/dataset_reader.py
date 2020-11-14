import re
import csv
import math


DELTA_TIMESTAMP_MS = 10     # each tick in the simulator is 0.01s



class Config:

    def __init__(self, filename, verbose=False):
        self.map = None
        self.StartframeTimestamp = None
        self.EndframeTimestamp = None
        self.EgoEndPositionX = None
        self.EgoEndPositionY = None
        self.TargetRightofWay = None

        self.read_from_file(filename)
        if verbose:
            self.print()
    
    def print(self):
        print("########## config info ##########")
        print("# map: ", self.map)
        print("# StartframeTimestamp: ", self.StartframeTimestamp)
        print("# EndframeTimestamp: ", self.EndframeTimestamp)
        print("# EgoEndPositionX: ", self.EgoEndPositionX)
        print("# EgoEndPositionY: ", self.EgoEndPositionY)
        print("# TargetRightofWay: ", self.TargetRightofWay)
        print()

    def read_from_file(self, filename):
        with open(filename, 'r') as fin:
            line  = fin.readline().strip()
            assert line[:4] == 'Map:'
            self.map = line[4:]

            while line[:25] != 'ReplayStartTimestamp(ms):':
                line = fin.readline().strip()
            self.StartframeTimestamp = int(line[25:])

            while(line[:22] != 'EndframeTimestamp(ms):'):
                line = fin.readline().strip()
            self.EndframeTimestamp = int(line[22:])

            line = fin.readline().strip()
            assert line[:15] == 'EgoEndPosition:'

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
                    assert line == "FinalState:id,s_now,s_tot"

                    while True:
                        line = fin.readline().strip()
                        if not line:
                            break
                        
                        info = list(line.strip().split(','))
                        assert len(info) == 3, info

                        t_id = int(info[0])
                        s_now = float(info[1])
                        s_tot = float(info[2])

                        assert t_id in self.track_dict, t_id
                        assert self.track_dict[t_id].agent_type == 'BehaveCar', self.track_dict[t_id].agent_type

                        self.track_dict[t_id].s_now = s_now
                        self.track_dict[t_id].s_tot = s_tot

                    for track_id in self.track_dict:
                        if self.track_dict[track_id].agent_type == 'BehaveCar' and self.track_dict[track_id].s_now == None:
                            self.track_dict[track_id].s_now = 1.0
                            self.track_dict[track_id].s_tot = 1.0

                    #for track_id in self.track_dict:
                    #    print(track_id, self.track_dict[track_id].s_now, self.track_dict[track_id].s_tot)

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
                        a = ms.velo - track.motion_states[time_stamp_ms - DELTA_TIMESTAMP_MS].velo
                        b = track.motion_states[time_stamp_ms - DELTA_TIMESTAMP_MS].velo - \
                            track.motion_states[time_stamp_ms - DELTA_TIMESTAMP_MS * 2].velo
                        ms.jerk = a - b
                    
                    track.motion_states[ms.time_stamp_ms] = ms
    

