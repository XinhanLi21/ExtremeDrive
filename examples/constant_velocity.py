"""Input-only smoke baseline. Does not access labels, map annotations or future actors."""
import re
import numpy as np
class Agent:
    def predict(self,inputs):
        match=re.search(r'Speed:\s*([0-9.]+)\s*m/s',inputs['language'])
        speed=float(match.group(1)) if match else 0.
        return np.column_stack((np.arange(1,9)*.5*speed,np.zeros(8),np.zeros(8)))
