import logging
from abc import ABCMeta, abstractmethod

import predictor.state


class Predictor:
    """Predictor .

    TODO: add more description
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def start(self):
        """Start

        TODO: add more detailed description
        """
        pass

    @abstractmethod
    def shutdown(self):
        """Shutdown

        TODO: add more detailed description
        """
        pass

    @abstractmethod
    def on_env(self, trajectory: predictor.state.Trajectory):
        """Receive environment state from remote server.

        TODO: add more detailed description
        """
        pass

    @abstractmethod
    def fetch_my_state(self) -> predictor.state.Trajectory:
        """Retrieve my state

        TODO: add more detailed description
        """
        pass


class DefaultPredictor(Predictor):
    def __init__(self, logger: logging.Logger):
        super().__init__()
        self._logger = logger

    def start(self):
        pass

    def shutdown(self):
        pass

    def on_env(self, trajectory: predictor.state.Trajectory):
        self._logger.info(f'predictor: get env\n==========')
        for state in trajectory.state():
            self._logger.info(f'track_id: {state.track_id}')
            self._logger.info(f'frame_id: {state.frame_id}')
            self._logger.info(f'timestamp_ms: {state.timestamp_ms}')
            self._logger.info(f'agent_type: {state.agent_type}')
            self._logger.info(f'x: {state.x}')
            self._logger.info(f'y: {state.y}')
            self._logger.info(f'vx: {state.vx}')
            self._logger.info(f'vy: {state.vy}')
            self._logger.info(f'psi_rad: {state.psi_rad}')
            self._logger.info(f'length: {state.length}')
            self._logger.info(f'width: {state.width}')

    def fetch_my_state(self) -> predictor.state.Trajectory:
        traj = predictor.state.Trajectory()
        state = predictor.state.State()
        state.track_id = 11
        state.frame_id = 21
        state.timestamp_ms = 31
        state.agent_type = "car1"
        state.x = 41
        state.y = 51
        state.vx = 61
        state.vy = 71
        state.psi_rad = 81
        state.length = 91
        state.width = 10
        traj.append_state(state)
        return traj
