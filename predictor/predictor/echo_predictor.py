from predictor.predictor import Predictor
from predictor.traj import *

import logging


class EchoPredictor(Predictor):
    def __init__(self, logger: logging.Logger):
        super().__init__()
        self._logger = logger
        self._last_state = None
    
    def start(self):
        pass

    def shutdown(self):
        pass

    def on_env(self, trajectory: Trajectory):
        self._logger.info(f'predictor: Receive from Simulator')
        assert len(trajectory.state()) == 10

        self._last_state = None
        for state in trajectory.state():
            self._logger.info(f'frame_id: {state.frame_id}; x: {state.x}; y: {state.y}')

            '''
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
            '''

            if self._last_state is not None:
                assert self._last_state.frame_id + 1 == state.frame_id, (self._last_state.frame_id, state.frame_id)

            self._last_state = state
        
        self._logger.info('\n')
        
    def fetch_my_state(self) -> Trajectory:
        self._logger.info(f'predictor: Echo the last fetched trajectory back to simulator\n')
        
        assert self._last_state is not None
        res = Trajectory()

        for _ in range(30):
            res.append_state(self._last_state)
        
        assert len(res.state()) == 30
        return res
