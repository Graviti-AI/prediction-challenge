#!/usr/bin/env python


DELTA_TIMESTAMP_MS = 100  # similar throughout the whole dataset


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

    def __str__(self):
        return 'x: %.3lf, y: %.3lf, vx: %.3lf, vy: %.3lf, psi_rad: %.3lf, lane_id: %d' % (self.x, self.y, self.vx, self.vy, self.psi_rad, self.lane_id)



class Measurements:

    #TODO:
    JERK_BOUNDARY = 0.0
    VELO_BOUNDARY = 0.0
    YAW_BOUNDARY  = 0.0

    def __init__(self, time_stamp_ms):
        assert isinstance(time_stamp_ms, int)
        self.time_stamp_ms = time_stamp_ms
        self.jerk = None
        self.velo = None
        self.delta_yaw = None
    
    def  __str__(self):
        return 'jerk: %.3lf, velo: %.3lf, delta_yaw: %.3lf' % (self.jerk, self.velo, self.delta_yaw)



class Track:
    def __init__(self, id):
        assert isinstance(id, int)
        self.track_id = id
        self.agent_type = None
        self.length = None
        self.width = None
        self.time_stamp_ms_first = None
        self.time_stamp_ms_last = None
        self.motion_states = dict()
        self.measurements = dict()

    def __str__(self):
        string = "Track: track_id=" + str(self.track_id) + ", agent_type=" + str(self.agent_type) + \
               ", length=" + str(self.length) + ", width=" + str(self.width) + \
               ", time_stamp_ms_first=" + str(self.time_stamp_ms_first) + \
               ", time_stamp_ms_last=" + str(self.time_stamp_ms_last) + \
               "\n motion_states:"
        for key, value in sorted(self.motion_states.items()):
            string += "\n    " + str(key) + ": " + str(value)
        #TODO:
        return string
