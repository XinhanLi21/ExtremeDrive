"""Current composite score. Comfort is category-dependent; EC is excluded."""
import numpy as np
from .marking_compliance import marking_compliance

def road_compliance(base, road):
    """Joint marking, drivable-surface and physical-boundary compliance."""
    checks = [marking_compliance(base), road.get('drivable_area_compliance')]
    exclusion = base.get('annotated_exclusion_compliance')
    if exclusion is not None:
        checks.append(exclusion)
    if any(v == 0 for v in checks):
        return 0.
    if any(v is None for v in checks):
        return None
    return float(np.prod(checks))

def scoped_total(base, road, comfort, applicable):
    gates = [base.get('no_collision'), road_compliance(base, road)]
    weighted = [(5, base.get('progress')), (5, base.get('ttc_compliance'))]
    if applicable:
        weighted.append((2, comfort))
    if any(v == 0 for v in gates):
        return 0.
    if any(v is None for v in gates) or any(v is None for _, v in weighted):
        return None
    return float(100 * np.prod(gates) * sum(w * v for w, v in weighted) / sum(w for w, v in weighted))
