# Go forward planner - dummy proof of concept

from planner.planner import Planner
from predictor.traj import *

import logging

class GoForwardPlanner(Planner):
    """Dummy planner that sends a simple Go forward message every time. """

    def __init__(self, logger: logging.Logger):
        self.traj = None
        self._logger = logger

    def on_env(self, map_name,
            reference_points: [],
            my_traj: Trajectory,
            other_trajs: [],
            obstacle_info: [],
            human_input=[],
            ):
        self._logger.info(f'planner: Receive from Simulator')
        self.traj = my_traj

    def fetch_my_plan(self) -> Trajectory:
        """Retrieve my trajectory"""
        self._logger.info(f'planner: Return my trajectory to the simulator\n')
        return self.traj
