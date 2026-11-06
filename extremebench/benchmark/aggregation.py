"""Current composite score. Comfort is category-dependent; EC is excluded."""
import numpy as np
from .marking_compliance import marking_compliance

def scoped_total(base, road, comfort, applicable):
    gates = [base.get('no_collision'), marking_compliance(base),
             1 if base.get('annotated_exclusion_compliance') is None else base['annotated_exclusion_compliance'],
             road.get('drivable_area_compliance'), road.get('driving_direction_compliance')]
    weighted = [(5, base.get('progress')), (5, base.get('ttc_compliance'))]
    if applicable:
        weighted.append((2, comfort))
    if any(v == 0 for v in gates):
        return 0.
    if any(v is None for v in gates) or any(v is None for _, v in weighted):
        return None
    return float(100 * np.prod(gates) * sum(w * v for w, v in weighted) / sum(w for w, v in weighted))
