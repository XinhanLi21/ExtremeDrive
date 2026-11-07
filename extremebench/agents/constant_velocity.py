import re
import numpy as np
from .base import PlanningAgent

class ConstantVelocityAgent(PlanningAgent):
    def predict(self, inputs):
        match = re.search(r"Speed:\s*([0-9.]+)\s*m/s", inputs["language"])
        speed = float(match.group(1)) if match else 0.0
        times = np.asarray(inputs.get('target_times_s', np.arange(1, 9) * 0.5))
        return np.column_stack((times * speed, np.zeros(len(times)), np.zeros(len(times))))
