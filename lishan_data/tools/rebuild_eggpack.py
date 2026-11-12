"""Remove source episode_index=35; stage and validate v3 before publication."""
from pathlib import Path
import os
os.environ.setdefault("HF_HOME", "/tmp/wamu-eggpack-hf")
os.environ.setdefault("HF_DATASETS_CACHE", "/tmp/wamu-eggpack-hf/datasets")
from datetime import datetime
import json, shutil, hashlib
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import av, torch
from lerobot.datasets.compute_stats import compute_episode_stats, aggregate_stats
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'wamu_real/eggpack_lerobot'
DST=ROOT/'wamu_real/eggpack_lerobot_rebuilding'
BACK=ROOT/'dataset_backups/eggpack_lerobot_before_remove_ep35'
assert not DST.exists() and not BACK.exists(), 'Refusing to overwrite previous work'
info=json.loads((SRC/'meta/info.json').read_text())
metadata=[pq.read_table(p) for p in sorted((SRC/'meta/episodes').rglob('*.parquet'))]
schema=metadata[0].schema
rows=sorted([r for t in metadata for r in t.to_pylist()],key=lambda r:r['episode_index'])
assert [r['episode_index'] for r in rows]==list(range(80))
removed=rows[35];assert removed['episode_index']==35
keys=[k for k,v in info['features'].items() if v['dtype']=='video']
(DST/'meta').mkdir(parents=True)
shutil.copy2(SRC/'meta/tasks.parquet',DST/'meta/tasks.parquet')
all_stats=[];mapping=[];offset=0;checks=[]
for old in rows:
 old_id=old['episode_index']
 if old_id==35:continue
 new_id=len(mapping);r=dict(old)
 path=SRC/info['data_path'].format(chunk_index=old['data/chunk_index'],file_index=old['data/file_index'])
 t=pq.read_table(path);t=t.filter(pa.compute.equal(t['episode_index'],old_id));n=len(t)
 assert n==old['length']
 for key,values in [('episode_index',np.full(n,new_id,dtype=np.int64)),('index',np.arange(offset,offset+n,dtype=np.int64))]:
  j=t.schema.get_field_index(key);t=t.set_column(j,t.schema.field(j),pa.array(values,type=t.schema.field(j).type))
 chunk,file=divmod(new_id,info['chunks_size'])
 target=DST/info['data_path'].format(chunk_index=chunk,file_index=file);target.parent.mkdir(parents=True,exist_ok=True);pq.write_table(t,target)
 r.update({'episode_index':new_id,'data/chunk_index':chunk,'data/file_index':file,'dataset_from_index':offset,'dataset_to_index':offset+n,'meta/episodes/chunk_index':chunk,'meta/episodes/file_index':file})
 stats={}
 for k,v in old.items():
  if k.startswith('stats/'):
   _,feat,stat=k.split('/',2);stats.setdefault(feat,{})[stat]=np.asarray(v)
 numeric={k:np.asarray(t[k].to_pylist()) for k in t.column_names if k in info['features'] and info['features'][k]['dtype']!='video'}
 stats.update(compute_episode_stats(numeric,info['features']))
 for feat,values in stats.items():
  for stat,value in values.items():r[f'stats/{feat}/{stat}']=value.tolist()
 all_stats.append(stats)
 for key in keys:
  pre='videos/'+key+'/'
  source=SRC/info['video_path'].format(video_key=key,chunk_index=old[pre+'chunk_index'],file_index=old[pre+'file_index'])
  assert old[pre+'from_timestamp']==0
  dest=DST/info['video_path'].format(video_key=key,chunk_index=chunk,file_index=file);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
  with av.open(str(dest)) as c:
   s=c.streams.video[0];assert s.frames==n and abs(float(s.average_rate)-info['fps'])<1e-6
  assert hashlib.sha256(source.read_bytes()).digest()==hashlib.sha256(dest.read_bytes()).digest()
  r[pre+'chunk_index']=chunk;r[pre+'file_index']=file
 epfile=DST/f'meta/episodes/chunk-{chunk:03d}/file-{file:03d}.parquet';epfile.parent.mkdir(parents=True,exist_ok=True)
 pq.write_table(pa.Table.from_pylist([r],schema=schema),epfile)
 mapping.append({'source_episode_index':old_id,'episode_index':new_id,'length':n});checks.append((old['dataset_from_index'],offset,n));offset+=n
 if new_id%10==0:print('rebuilt',new_id,flush=True)
info['total_episodes']=len(mapping);info['total_frames']=offset;info['splits']={'train':f'0:{len(mapping)}'}
(DST/'meta/info.json').write_text(json.dumps(info,indent=2)+'\n')
aggregated=aggregate_stats(all_stats)
(DST/'meta/stats.json').write_text(json.dumps({k:{s:v.tolist() for s,v in vals.items()} for k,vals in aggregated.items()},indent=2)+'\n')
print('Loading both datasets for comparison',flush=True)
old_ds=LeRobotDataset('local/eggpack-original',root=SRC,video_backend='pyav')
new_ds=LeRobotDataset('local/eggpack-cleaned',root=DST,video_backend='pyav')
assert len(new_ds)==offset and new_ds.num_episodes==79
for old_start,new_start,n in checks:
 for delta in [0,n-1]:
  before=old_ds[old_start+delta];after=new_ds[new_start+delta]
  for key in keys+['action','observation.state','timestamp','frame_index']:
   assert torch.equal(before[key],after[key]),(new_start,delta,key)
print('All episode boundary samples verified',flush=True)
assert len(list((DST/'videos').rglob('*.mp4')))==79*3
report={'removed_source_episode_index':35,'retained_source_episode_34':True,'removed_frames':removed['length'],'episodes':79,'frames':offset,'mapping':mapping,'validation':'LeRobot v3 loader; every retained episode first/last frame including all 3 decoded cameras, action/state/timing compared exactly; all video frame counts checked and copied files SHA256 matched','stats':'Numeric episode stats recomputed; unchanged per-episode image stats retained; global stats aggregated using installed LeRobot functions','backup':str(BACK)}
(DST/'curation_report.json').write_text(json.dumps(report,indent=2)+'\n')
BACK.parent.mkdir(exist_ok=True);SRC.rename(BACK)
try:DST.rename(SRC)
except BaseException:
 BACK.rename(SRC);raise
print('PUBLISHED',json.dumps({k:v for k,v in report.items() if k!='mapping'}),flush=True)
