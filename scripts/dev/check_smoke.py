"""Compare three newly executed trajectories against frozen reference records."""
from pathlib import Path
import json
import numpy as np
from extremebench.toolkit.data import load
ROOT=Path(__file__).resolve().parents[2]
reference={r['token']:r for r in load(ROOT/'assets/reference/pluto/results.json.gz')}
paths=list((ROOT/'results/smoke/scores').glob('*.json'))
assert len(paths)==3,'Expected exactly three smoke windows'
for path in paths:
 result=json.loads(path.read_text());old=reference[result['token']]
 np.testing.assert_allclose(result['trajectory'],old['trajectory'],rtol=0,atol=1e-8)
 np.testing.assert_allclose(result['score'],old['score'],rtol=0,atol=1e-8)
 for field in ['no_collision','marking_compliance','ttc_compliance']:
  assert result['metrics'][field]==old['metrics'][field],(result['token'],field)
print('Three fresh trajectories, scores and safety gates match frozen references.')
