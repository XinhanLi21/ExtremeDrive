"""Scene-specific empirical road friction for execution, separate from planner inputs."""
from pathlib import Path
from hashlib import sha256
import json
import math


def table_identity(assets):
    return sha256((Path(assets)/'scene_friction.json').read_bytes()).hexdigest()


def resolve_friction(assets, context, token):
    path=Path(assets)/'scene_friction.json'
    if not path.is_file():
        raise ValueError('Scene friction assignments are required: '+str(path))
    payload=path.read_bytes();table=json.loads(payload)
    if table.get('schema')!='scene-friction-v1':
        raise ValueError('Unsupported scene friction schema')
    episode=table.get('execution_clips',{}).get(token)
    if episode is None:raise ValueError('Missing scene friction binding for '+token)
    expected=context.get('row',{}).get('context',{}).get('source_episode')
    audited=context.get('audit',{}).get('source_episode')
    if any(e is not None and e!=episode for e in (expected,audited)):
        raise ValueError('Scene friction source episode does not match execution context')
    row=table.get('records',{}).get(episode)
    if not isinstance(row,dict):raise ValueError('Missing friction assignment for '+episode)
    surface=row.get('surface');mu=row.get('mu')
    if surface not in ('snow','wet','dry'):raise ValueError('Unknown road surface for '+episode)
    if type(mu) not in (int,float) or not math.isfinite(mu) or mu<=0:
        raise ValueError('Friction must be a finite positive number for '+episode)
    if mu!=table.get('surface_mu',{}).get(surface):
        raise ValueError('Friction assignment disagrees with the road-condition mapping')
    return {'mu':float(mu),'surface':surface,'source_episode':episode,
            'basis':table['basis'],'classification_basis':row.get('classification_basis'),
            'table_sha256':sha256(payload).hexdigest()}
