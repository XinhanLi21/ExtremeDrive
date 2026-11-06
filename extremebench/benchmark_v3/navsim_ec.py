"""Load traceable outputs of the pinned, unmodified upstream EC simulator."""
import hashlib,json
PIN='c0b6fb05d9060f665dc98830285fe236312dc2ba'
def load_ec(root,model_dir,horizon):
    path=root/'outputs/navsim_ec_20261019'
    manifest=json.loads((path/'provenance.json').read_text())
    assert manifest['upstream_commit']==PIN, 'Unexpected NAVSIM revision'
    digest=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    assert digest(model_dir/'predictions_4s.jsonl')==manifest['predictions_sha256'][model_dir.name], 'Stale EC predictions: rerun run_navsim_ec.py'
    assert digest(path/'inputs.json')==manifest['inputs_sha256'], 'EC input manifest changed'
    rows=json.loads((path/f'{model_dir.name}_{horizon:g}s_ec.json').read_text())
    result={r['window_id']:r for r in rows}
    assert len(result)==len(rows)
    tokens={r['token'][:-5] for r in json.loads((model_dir/'test.json').read_text()) if r['token'].endswith(':main')}
    assert set(result)==tokens
    assert all(r['score'] in (0.,1.,None) for r in rows)
    return result
