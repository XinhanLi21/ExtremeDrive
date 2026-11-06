"""Annotation-conditional emergency safety, not full maneuver completion."""

def emergency_safety(row):
    category = str(row.get('category', '')).lower()
    if not any(k in category for k in ('critical', 'emergency', 'evasive')):
        return {'applicable': False, 'success': None}
    m, road = row['metrics'], row['road_metrics']
    checks = {k: m.get(k) for k in ('no_collision', 'marking_compliance', 'ttc_compliance')}
    checks.update({k: road.get(k) for k in ('drivable_area_compliance', 'driving_direction_compliance')})
    # Exclusion geometry is conditional on annotation coverage; disclose absence.
    if m.get('annotated_exclusion_compliance') is not None:
        checks['annotated_exclusion_compliance'] = m['annotated_exclusion_compliance']
    success = False if any(v is not None and v < 1 for v in checks.values()) else None if any(v is None for v in checks.values()) else True
    return {'applicable': True, 'success': success, 'checks': checks,
            'exclusion_available': m.get('annotated_exclusion_compliance') is not None,
            'maneuver_completion_success': None,
            'scope': 'annotated safety within prediction horizon; comfort excluded; task endpoint and measured dynamics limits unavailable'}
