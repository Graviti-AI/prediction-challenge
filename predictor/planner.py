import logging
from abc import ABCMeta, abstractmethod

from predictor.traj import *

class Planner:
    """Planner Class. Users wanting to implement their own planning algorithms
    can extend this class then override the receive_info() and
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    @abstractmethod
    def start(self):
        """Start. Perform any thread-specific setup here.
        """
        pass

    @abstractmethod
    def shutdown(self):
        """Shutdown. Perform any thread-specific tear-down here.
        """
        pass

    @abstractmethod
    def on_env(self, map_name, my_traj: Trajectory, other_trajs: []):
        """Receive environment state from remote server. We can use this to
        generate a plan, which should be saved as a state variable.
        """
        pass

    @abstractmethod
    def fetch_my_plan(self) -> Trajectory:
        """Retrieve my plan, and transfer control back to the server.
        Returns a Trajectory of my planned actions
        """
        pass
