class State:
    def __init__(self, pt=None):
        if pt:
            self.track_id = pt.track_id
            self.frame_id = pt.frame_id
            self.timestamp_ms = pt.timestamp_ms
            self.agent_type = pt.agent_type
            self.x = pt.x
            self.y = pt.y
            self.vx = pt.vx
            self.vy = pt.vy
            self.psi_rad = pt.psi_rad
            self.length = pt.length
            self.width = pt.width
            self.jerk = pt.jerk
            self.current_lanelet_id = pt.current_lanelet_id
            self.s_of_current_lanelet = pt.s_of_current_lanelet
            self.d_of_current_lanelet  = pt.d_of_current_lanelet
        else:
            self.track_id = 0
            self.frame_id = 0
            self.timestamp_ms = 0
            self.agent_type = 'car'
            self.x = 0.0
            self.y = 0.0
            self.vx = 0.0
            self.vy = 0.0
            self.psi_rad = 0.0
            self.length = 0.0
            self.width = 0.0
            self.jerk = 0.0
            self.current_lanelet_id = 0
            self.s_of_current_lanelet = 0.0
            self.d_of_current_lanelet  = 0


class Trajectory:
    def __init__(self):
        self._trajectory = []

    @property
    def states(self):
        return self._trajectory

    def append_state(self, state: State):
        self._trajectory.append(state)

    def state(self):
        return self._trajectory
