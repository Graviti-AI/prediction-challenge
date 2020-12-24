import logging
from abc import ABCMeta, abstractmethod

from predictor.traj import *


class MyState:
    def __init__(self, trajectories, probabilities):
        self._trajectories = trajectories
        self._probabilities = probabilities
        pass

    @property
    def trajectories(self) -> []:
        return self._trajectories

    @property
    def probabilities(self) -> []:
        return self._probabilities


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
    def on_env(self, map_name, my_traj: Trajectory, other_trajs: []):
        """Receive environment state from remote server.

        TODO: add more detailed description
        """
        pass

    @abstractmethod
    def fetch_my_state(self) -> MyState:
        """Retrieve my state

        TODO: add more detailed description
        """
        pass
