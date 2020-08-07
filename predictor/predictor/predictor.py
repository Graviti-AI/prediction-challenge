import logging
from abc import ABCMeta, abstractmethod

from predictor.state import *


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
    def on_env(self, trajectory: Trajectory):
        """Receive environment state from remote server.

        TODO: add more detailed description
        """
        pass

    @abstractmethod
    def fetch_my_state(self) -> Trajectory:
        """Retrieve my state

        TODO: add more detailed description
        """
        pass
