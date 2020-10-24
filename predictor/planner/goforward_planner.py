# Go forward planner - dummy proof of concept

from planner.planner import Planner
from predictor.traj import *

import logging

class GoForwardPlanner(Planner):
    """Dummy planner that sends a simple Go forward message every time. """

    def __init__(self, logger: logging.Logger):
        self.traj = None

    def on_env(self, map_name,
            my_traj: Trajectory,
            other_trajs: [],
            human_input=[],
            obstacle_info=[],
            #reference_path: Trajectory,
            ):
        self.traj = my_traj

    def fetch_my_plan(self) -> Trajectory:
        """Retrieve my trajectory"""
        return self.traj
