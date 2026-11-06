"""Continuous EC severity diagnostic; never advertised as official NAVSIM EC."""
import math

def continuous_ec(record):
    if record.get('score') is None:return None
    rms=record['rms'];limits=record['thresholds']
    ratios=[float(rms[k])/float(limits[k]) for k in ('acceleration','jerk','yaw_rate','yaw_accel')]
    if not all(math.isfinite(r) and r>=0 for r in ratios):raise ValueError('finite nonnegative RMS required')
    # Same severity mapping as comfort v4. Worst channel prevents dilution.
    return max(1.,max(ratios))**-2
