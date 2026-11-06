"""Minimal public planning interface; no evaluator labels are supplied."""
from abc import ABC, abstractmethod
from typing import Mapping, Any
import numpy as np

class PlanningAgent(ABC):
    @abstractmethod
    def predict(self, inputs: Mapping[str, Any]) -> np.ndarray:
        """Return eight [x_m, y_m, yaw_rad] poses at 0.5 through 4.0 seconds."""
        raise NotImplementedError
