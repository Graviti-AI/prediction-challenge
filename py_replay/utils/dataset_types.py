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
        self.centerline = None

    def __str__(self):
        return 'x: %.3lf, y: %.3lf, vx: %.3lf, vy: %.3lf, psi_rad: %.3lf, lane_id: %d, centerline: %.3lf' % (self.x, self.y, self.vx, self.vy, self.psi_rad, self.lane_id, self.centerline)



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
        self.yaw2lane = None
    
    def  __str__(self):
        return 'jerk: %.3lf, velo: %.3lf, yaw2lane: %.3lf' % (self.jerk, self.velo, self.yaw2lane)



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
