import logging
import time

import grpc

import predictor.predictor
import predictor.traj
import simulator_pb2
import simulator_pb2_grpc


class SimulatorClient:
    def __init__(self, logger: logging.Logger, server_address, user_predictor: predictor.predictor.Predictor):
        self._logger = logger
        self._server_address = server_address
        self._client = None
        self._stopped = False
        self._predictor = user_predictor

        self._simulator_paused = False

    def start(self, loop_interval):
        with grpc.insecure_channel(self._server_address) as channel:
            self._client = simulator_pb2_grpc.SimulatorServerStub(channel)
            next_loop = time.perf_counter()
            while True:
                try:
                    self.fetch_env()
                except Exception as e:
                    self._logger.warning(f'failed to connect to remote server')
                    self._logger.warning(e.__str__())
                    self._logger.warning(f'will try again 5 seconds later')
                    time.sleep(5)

                if self._simulator_paused:
                    self.report_state()
                else:
                    try:
                        self.report_state()
                    except Exception as e:
                        self._logger.warning(f'failed to connect to remote server')
                        self._logger.warning(e.__str__())
                        self._logger.warning(f'will try again 5 seconds later')
                        time.sleep(5)

                curr = time.perf_counter()
                interval = max(0, next_loop + loop_interval - curr)
                next_loop = curr + interval
                time.sleep(interval)

    def shutdown(self):
        self._stopped = True

    @staticmethod
    def _proto_traj_to_traj(proto_traj):
        trajectory = predictor.traj.Trajectory()
        for pt in proto_traj.state:
            trajectory.append_state(predictor.traj.State(pt))
        return trajectory

    def fetch_env(self):
        response = self._client.FetchEnv(simulator_pb2.FetchEnvRequest())
        if response.resp_code == 0:
            map_name = response.map_name
            my_traj = self._proto_traj_to_traj(response.my_traj)
            other_trajs = []
            for other_traj in response.other_trajs:
                other_trajs.append(self._proto_traj_to_traj(other_traj))

            self._predictor.on_env(map_name, my_traj, other_trajs)
        elif response.resp_code == 233:     # the simulator paused
            self._simulator_paused = True
            print(f'resp_code={response.resp_code}, the simulator paused')
        else:
            self._logger.warning(f'fetch_env failed, resp_code={response.resp_code}')

    def report_state(self):
        req = simulator_pb2.PushMyTrajectoryRequest()

        if self._simulator_paused:
            try:
                resp = self._client.PushMyTrajectory(req)
                # send an empty request to inform the simulator that the client has quit
            except Exception as e:
                print('Close Predictor')
                exit(0)

        my_state = self._predictor.fetch_my_state()
        for trajs in my_state.trajectories:
            traj = req.pred_trajs.add()
            for state in trajs.states:
                pt = traj.state.add()
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
                pt.jerk = state.jerk
                pt.current_lanelet_id = state.current_lanelet_id
                pt.s_of_current_lanelet = state.s_of_current_lanelet
                pt.d_of_current_lanelet = state.d_of_current_lanelet

        for probability in my_state.probabilities:
            req.probability.append(probability)

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
