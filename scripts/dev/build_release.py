"""Build a deterministic GitHub source archive with compact benchmark assets."""
from pathlib import Path
import hashlib,zipfile,re
ROOT=Path(__file__).resolve().parents[2]
version=re.search(r'^version = "([^"]+)"', (ROOT/'pyproject.toml').read_text(),re.M).group(1)
out=ROOT/'dist';out.mkdir(exist_ok=True);dest=out/f'extremedrive-toolkit-{version}-github.zip'
exclude={'.git','.venv','__pycache__','build','dist','.pytest_cache'}
with zipfile.ZipFile(dest,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in sorted(ROOT.rglob('*')):
  rel=p.relative_to(ROOT)
  if not p.is_file() or p.is_symlink() or rel.parts[0] in ('data','results') or any(s in exclude or s.endswith('.egg-info') for s in rel.parts) or p.suffix=='.pyc':continue
  info=zipfile.ZipInfo(f'extremedrive-toolkit-{version}/'+rel.as_posix(),date_time=(2026,11,6,0,0,0))
  info.compress_type=zipfile.ZIP_DEFLATED;info.external_attr=(0o100755 if p.suffix=='.sh' else 0o100644)<<16
  z.writestr(info,p.read_bytes())
with zipfile.ZipFile(dest) as z:
 if z.testzip() is not None:raise RuntimeError('archive verification failed')
sha=hashlib.sha256(dest.read_bytes()).hexdigest();dest.with_suffix('.zip.sha256').write_text(sha+'  '+dest.name+'\n')
print(f'{dest}\n{dest.stat().st_size} bytes\nSHA256 {sha}')
