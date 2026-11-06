import unittest,tempfile,json,tarfile,io
from pathlib import Path
from extremebench.toolkit.data import Dataset,read_predictions,prediction_digest
from extremebench.toolkit.download import plan,extract_episode
from extremebench.toolkit.evaluation import summarize
ROOT=Path(__file__).resolve().parents[1];ASSETS=ROOT/'assets'
class ToolkitTests(unittest.TestCase):
 def test_split_and_no_label_leak(self):
  for split,count in [('train',758),('val',89),('test',341)]:
   ds=Dataset(ASSETS,'/portable/data',split);self.assertEqual(len(ds.tokens()),count)
   inp=ds.inputs(ds.tokens()[0]);self.assertNotIn('target',inp);self.assertNotIn('context',inp);self.assertTrue(all(p.startswith('/portable/data/') for p in inp['images']))
  self.assertEqual(len(ds.tokens(True)),131)
 def test_download_coverage(self):
  cfg,records=plan(ASSETS,'execution');self.assertEqual(len(records),13);self.assertEqual(len(cfg['revision']),40)
  _,records=plan(ASSETS,'all');self.assertEqual(len(records),336)
 def test_archive_escape_and_links(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'bad.tar'
   for name,kind in [('../escape',tarfile.REGTYPE),('link',tarfile.SYMTYPE)]:
    with tarfile.open(p,'w') as t:
     member=tarfile.TarInfo(name);member.type=kind;member.linkname='/tmp';t.addfile(member)
    with self.assertRaises(ValueError):extract_episode(p,Path(d)/'out','episode')
 def test_archive_root(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'good.tar'
   with tarfile.open(p,'w') as t:
    m=tarfile.TarInfo('episode/images/a.jpg');m.size=3;t.addfile(m,io.BytesIO(b'abc'))
   extract_episode(p,Path(d)/'out','episode');self.assertEqual((Path(d)/'out/images/a.jpg').read_bytes(),b'abc')
 def test_invalid_submission(self):
  p={'token':'token','times_s':[.5*i for i in range(1,9)],'poses':[[0,0,0]]*8}
  with tempfile.TemporaryDirectory() as d:
   f=Path(d)/'p.jsonl';f.write_text(json.dumps(p)+'\n'+json.dumps(p))
   with self.assertRaises(ValueError):read_predictions(f)
   p['poses'][0]=[float('nan'),0,0];f.write_text(json.dumps(p))
   with self.assertRaises(ValueError):read_predictions(f)
 def test_unknown_not_dropped(self):
  s=summarize([{'score':100},{'score':None}]);self.assertIsNone(s['metrics']['score']['mean']);self.assertEqual(s['metrics']['score']['lower'],50);self.assertEqual(s['metrics']['score']['upper'],100)
 def test_review_binding_changes_on_prediction(self):
  p={'token':'x','times_s':[.5*i for i in range(1,9)],'poses':[[0,0,0]]*8};a=prediction_digest(p);p['poses'][0]=[1,0,0];self.assertNotEqual(a,prediction_digest(p))
if __name__=='__main__':unittest.main()
