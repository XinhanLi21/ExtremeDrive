"""Minimal public planning interface; no evaluator labels are supplied."""
from abc import ABC, abstractmethod
from typing import Mapping, Any
import numpy as np

class PlanningAgent(ABC):
    @abstractmethod
    def predict(self, inputs: Mapping[str, Any]) -> np.ndarray:
        """Return [x_m, y_m, yaw_rad] at inputs['target_times_s'] (8 or 16 poses)."""
        raise NotImplementedError
