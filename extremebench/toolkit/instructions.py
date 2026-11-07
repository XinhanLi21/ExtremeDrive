"""Input policies: scenario annotations are not verified online route commands."""
import re
POLICIES = ('sensor_only', 'navigation_only', 'scenario_hint', 'no_maneuver_hint')
# Annotation normalization only; these entries are never declared causal navigation.
NAVIGATION = {
    'Drive straight ahead': 'straight', 'Turn left': 'left', 'Turn right': 'right',
    'Change lane to the left': 'lane_change_left', 'Change lane to the right': 'lane_change_right',
    'Waiting to turn left': 'wait_left',
}

def annotation(row):
    text=row['language'];match=re.search(r'Instruction:\s*(.*?)\.\s*Predict the next',text)
    instruction=match.group(1).strip().rstrip('.') if match else ''
    emergency=bool(re.search(r'emergen|avoid|collision|evasive|obstacle',instruction,re.I))
    speed=re.search(r'Speed:\s*([-+0-9.]+)\s*m/s',text)
    return {'source_field':'aligned episode metadata.jsonl: prompt',
            'source_episode':row.get('context',{}).get('source_episode'),
            'capture_timestamp_s':row.get('time_alignment',{}).get('capture_timestamp_s'),
            'annotation_created_at':None, 'online_available_at_capture':None,
            'causal_navigation_verified':False,
            'navigation_annotation':NAVIGATION.get(instruction) if not emergency else None,
            'maneuver_hint':instruction if emergency else None,
            'scenario_annotation':instruction,
            'speed_mps':float(speed.group(1)) if speed else None}

def language_inputs(row, policy='sensor_only'):
    if policy not in POLICIES:raise ValueError('Unknown input policy: '+policy)
    a=annotation(row)
    # No original instruction, category, folder name or privileged hint is returned in the text fields.
    prefix=f"Speed: {a['speed_mps']:.2f} m/s. " if a['speed_mps'] is not None else ''
    if policy=='scenario_hint' or (policy=='no_maneuver_hint' and a['maneuver_hint'] is None):text=row['language']
    else:text=prefix+'Predict the next 4 seconds of driving.'
    # There is no timestamped online-navigation source in the current release.
    # navigation_only therefore stays neutral instead of guessing from scenario names.
    return dict(language=text,input_policy=policy,navigation_command=None,navigation_valid=False,
                navigation_source='unavailable',speed_mps=a['speed_mps'])
