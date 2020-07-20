import logging
import time

import grpc

import predictor.predictor
import predictor.state
import simulator_pb2
import simulator_pb2_grpc


class SimulatorClient:
    def __init__(self, logger: logging.Logger, server_address, user_predictor: predictor.predictor.Predictor):
        self._logger = logger
        self._server_address = server_address
        self._client = None
        self._stopped = False
        self._predictor = user_predictor

    def start(self, loop_interval):
        with grpc.insecure_channel(self._server_address) as channel:
            self._client = simulator_pb2_grpc.SimulatorServerStub(channel)
            next_loop = time.perf_counter()
            while True:
                try:
                    self.fetch_env()
                    self.report_state()
                except Exception as e:
                    self._logger.warning(f'failed to connect to remote server')
                    self._logger.warning(e.__str__())
                    self._logger.warning(f'will try again 2 seconds later')
                    time.sleep(2)
                curr = time.perf_counter()
                interval = max(0, next_loop + loop_interval - curr)
                next_loop = curr + interval
                time.sleep(interval)

    def shutdown(self):
        self._stopped = True

    def fetch_env(self):
        response = self._client.FetchEnv(simulator_pb2.FetchEnvRequest())
        if response.resp_code == 0:
            trajectory = predictor.state.Trajectory()
            for pt in response.trajectory.state:
                trajectory.append_state(predictor.state.State(pt))
            self._predictor.on_env(trajectory)
        else:
            self._logger.warning(f'fetch_env failed, resp_code={response.resp_code}')

    def report_state(self):
        req = simulator_pb2.PushMyTrajectoryRequest()

        for state in self._predictor.fetch_my_state().trajectory:
            pt = req.trajectory.state.add()
            pt.track_id = state.track_id
            pt.frame_id = state.frame_id
            pt.timestamp_ms = state.timestamp_ms
            pt.agent_type = state.agent_type
            pt.x = state.x
            pt.y = state.y
            pt.vx = state.vx
            pt.vy = state.vy
            pt.psi_rad = state.psi_rad
            pt.length = state.length
            pt.width = state.width

        resp = self._client.PushMyTrajectory(req)
        if resp.resp_code != 0:
            self._logger.warning(f'report_state failed, resp_code={resp.resp_code}')

    @property
    def stopped(self):
        self._predictor.shutdown()
        return self._stopped

    @property
    def loop_interval(self):
        return self._loop_interval
