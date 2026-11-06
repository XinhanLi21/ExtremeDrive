"""Pinned Hugging Face episode retrieval; archives are data, never executable code."""
from pathlib import Path,PurePosixPath
import tarfile,shutil,json
from .data import load,Dataset

def plan(assets,scope):
    cfg=load(Path(assets)/'huggingface.json');files=set(cfg['files']);episodes=set()
    splits=['train','val','test'] if scope=='all' else ['test'] if scope=='execution' else [scope]
    for split in splits:
        ds=Dataset(assets,split=split)
        for token in ds.tokens(execution=scope=='execution'):episodes.add(ds.rows[token]['context']['source_episode'])
    selected=[];missing=[]
    for ep in sorted(episodes):
        archive=ep+'.tar'
        # Some episodes are expanded directories, others are per-episode tar files.
        if archive in files:selected.append({'episode':ep,'files':[archive],'archive':True})
        else:
            matches=sorted(f for f in files if f.startswith(ep+'/'))
            if matches:selected.append({'episode':ep,'files':matches,'archive':False})
            else:missing.append(ep)
    if missing:raise ValueError('Pinned HF revision lacks source episodes: '+str(missing))
    return cfg,selected

def extract_episode(archive,dest,episode_name):
    dest=Path(dest);dest.mkdir(parents=True,exist_ok=True)
    with tarfile.open(archive,'r:*') as tar:
        for member in tar:
            raw=PurePosixPath(member.name)
            if raw.is_absolute() or '..' in raw.parts or member.issym() or member.islnk() or not(member.isfile() or member.isdir()):raise ValueError('unsafe archive entry: '+member.name)
            parts=list(raw.parts)
            if parts and parts[0]==episode_name:parts=parts[1:]
            if not parts:continue
            path=dest.joinpath(*parts)
            if not path.resolve().is_relative_to(dest.resolve()):raise ValueError('unsafe extraction destination')
            if member.isdir():path.mkdir(parents=True,exist_ok=True);continue
            path.parent.mkdir(parents=True,exist_ok=True)
            if path.is_symlink():raise ValueError('refusing symlink destination')
            with tar.extractfile(member) as source,path.open('wb') as target:shutil.copyfileobj(source,target)

def download(assets,root,scope='execution',dry_run=False):
    cfg,selected=plan(assets,scope)
    if dry_run:return {'repo_id':cfg['repo_id'],'revision':cfg['revision'],'episodes':len(selected),'files':selected,'license':cfg['license']}
    from huggingface_hub import hf_hub_download
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    for record in selected:
        for name in record['files']:
            local=hf_hub_download(repo_id=cfg['repo_id'],repo_type='dataset',revision=cfg['revision'],filename=name)
            if record['archive']:extract_episode(local,root/record['episode'],Path(record['episode']).name)
            else:
                target=root/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(local,target)
    (root/'extremedrive_download.json').write_text(json.dumps({'repo_id':cfg['repo_id'],'revision':cfg['revision'],'scope':scope,'episodes':[r['episode'] for r in selected]},indent=2))
    return {'episodes':len(selected),'dataroot':str(root),'revision':cfg['revision']}
