from predictor.predictor import Predictor
from predictor.state import *

import logging


class EchoPredictor(Predictor):
    def __init__(self, logger: logging.Logger):
        super().__init__()
        self._logger = logger
        self._last_fetch_traj = None

    def start(self):
        pass

    def shutdown(self):
        pass

    def on_env(self, trajectory: Trajectory):
        self._logger.info(f'predictor: Receive from Simulator\n==========')

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
        
        self._last_fetch_traj = trajectory

    def fetch_my_state(self) -> Trajectory:
        self._logger.info(f'\n==========\npredictor: Echo the last fetched trajectory back to simulator\n\n')
        
        assert self._last_fetch_traj is not None
        res = self._last_fetch_traj

        self._last_fetch_traj = None
        return res
