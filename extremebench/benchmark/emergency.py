"""Annotation-conditional emergency safety, not full maneuver completion."""

from .aggregation import road_compliance

def emergency_safety(row):
    category = str(row.get('category', '')).lower()
    if not any(k in category for k in ('critical', 'emergency', 'evasive')):
        return {'applicable': False, 'success': None}
    m, road = row['metrics'], row['road_metrics']
    checks = {k: m.get(k) for k in ('no_collision', 'ttc_compliance')}
    checks['road_compliance'] = road_compliance(m, road)
    success = False if any(v is not None and v < 1 for v in checks.values()) else None if any(v is None for v in checks.values()) else True
    return {'applicable': True, 'success': success, 'checks': checks,
            'exclusion_available': m.get('annotated_exclusion_compliance') is not None,
            'maneuver_completion_success': None,
            'scope': 'annotated safety within prediction horizon; comfort excluded; task endpoint and measured dynamics limits unavailable'}
