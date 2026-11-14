"""Critical success: no collision, road compliance and a separate short-horizon TTC check."""

from .aggregation import road_compliance

EMERGENCY_SUCCESS_DEFINITION = "critical_collision_road_ttc05_v1"

def emergency_safety(row):
    category = str(row.get('category', '')).lower()
    if not any(k in category for k in ('critical', 'emergency', 'evasive')):
        return {'applicable': False, 'success': None}
    m, road = row['metrics'], row['road_metrics']
    checks = {k: m.get(k) for k in ('no_collision', 'emergency_ttc_compliance')}
    checks['road_compliance'] = road_compliance(m, road)
    success = False if any(v is not None and v < 1 for v in checks.values()) else None if any(v is None for v in checks.values()) else True
    return {'applicable': True, 'success': success, 'checks': checks,
            'definition': EMERGENCY_SUCCESS_DEFINITION,
            'ttc_horizon_s': row.get('emergency_ttc_horizon_s', 0.5),
            'exclusion_available': m.get('annotated_exclusion_compliance') is not None,
            'maneuver_completion_success': None,
            'scope': 'Collision avoidance, road compliance and emergency TTC compliance over the prediction horizon; comfort excluded'}
