# Planning agents

An agent receives causal inputs and returns future poses on the requested sampling grid. It owns model loading, device placement and inference; the toolkit owns data selection and evaluation.

```python
import numpy as np
from extremebench.agents.base import PlanningAgent

class Agent(PlanningAgent):
    def __init__(self):
        # Load your checkpoint once here.
        pass

    def predict(self, inputs):
        # inputs: token, images, image_history, language, history, history_valid, target_times_s
        # Return shape (len(inputs["target_times_s"]), 3), x forward [m], y left [m], yaw CCW [rad].
        return your_model(inputs)
```

```bash
extremedrive --assets assets predict --agent my_agent:Agent \
  --dataroot data --execution-only --out predictions.jsonl
```

The class must be importable and accept a zero-argument constructor. Load checkpoint/config paths inside your adapter or from explicit environment variables. A GPU framework is not a toolkit dependency.

The bundled input-only baseline is `extremebench.agents.constant_velocity:ConstantVelocityAgent`. It uses the current speed in the instruction and assumes straight constant-speed motion. It provides a deterministic baseline for testing data and evaluation interfaces.

Future targets, map/evaluator contexts and actor observations are not passed to `predict`. `Dataset.target()` is a separate supervised-label accessor. Remove `--execution-only` to predict the full split for displacement evaluation. Model training is managed by the individual planning method.

Input and output sampling are independently configurable at 2 Hz or 4 Hz. See [sampling](sampling.md) for timestamps, valid masks and grid conventions.

Default `language` contains measured speed and a generic prediction request only. Source annotations are available separately for auditing, never as default online navigation. See [input policy](input_policy.md).
