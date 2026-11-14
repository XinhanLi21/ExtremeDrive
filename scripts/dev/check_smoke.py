"""Check that the smoke run produced complete, valid fresh evaluation outputs."""
from pathlib import Path
import json
import numpy as np
from extremebench.toolkit.data import read_predictions, safe_token
ROOT=Path(__file__).resolve().parents[2]
predictions=read_predictions(ROOT/'examples/smoke_predictions.jsonl')
out=ROOT/'results/smoke'
paths=list((out/'scores').glob('*.json'))
records=[json.loads(p.read_text()) for p in paths]
assert len(records)==len(predictions),'Smoke output count differs from input'
assert {r['token'] for r in records}==set(predictions),'Smoke token coverage mismatch'
for result in records:
    assert result['status']=='automated',result['token']
    assert result['manual_review_applied'] is False
    assert result['score'] is None or 0 <= result['score'] <= 100
    trajectory=np.asarray(result['trajectory'],dtype=float)
    assert len(trajectory)>1 and np.isfinite(trajectory).all()
    assert np.all(np.diff(trajectory[:,0])>0)
    rollout=json.loads((out/'rollouts'/(safe_token(result['token'])+'.json')).read_text())
    assert not rollout['failure'],rollout['failure']
    assert rollout['source_prediction']['token']==result['token']
summary=json.loads((out/'summary.json').read_text())
assert summary['windows']==len(predictions)
assert summary['mode']=='fresh_automated_execution'
assert not summary['complete_cohort'],'Smoke examples must remain a diagnostic subset'
assert (out/'index.html').is_file() and (out/'metrics.csv').is_file()
print('Fresh smoke output coverage, trajectories, execution status and report verified.')
