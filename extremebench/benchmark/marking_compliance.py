"""Combined marking gate: any violation fails; unresolved checks stay unknown."""
def marking_compliance(metrics):
    values=[metrics.get('white_compliance'),metrics.get('yellow_compliance')]
    if any(v==0 for v in values):return 0
    if any(v is None for v in values):return None
    return float(values[0]*values[1])
