"""Convert all 30-Hz wamu_real datasets to 15 Hz, validating before publication."""
import os
os.environ.setdefault('HF_HOME','/tmp/wamu-15hz-hf')
os.environ.setdefault('HF_DATASETS_CACHE','/tmp/wamu-15hz-hf/datasets')
from pathlib import Path
import json, shutil, subprocess
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import av, torch
from lerobot.datasets.compute_stats import (compute_episode_stats,aggregate_stats,sample_indices,auto_downsample_height_width,get_feature_stats)
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ROOT=Path(__file__).resolve().parents[1]

def convert(src):
 info=json.loads((src/'meta/info.json').read_text());assert info['fps']==30
 dst=ROOT/'dataset_staging'/f'{src.name}_15hz';backup=ROOT/'dataset_backups'/f'{src.name}_before_30_to_15hz'
 assert not dst.exists() and not backup.exists(),'Refusing to overwrite existing stage or backup'
 dst.mkdir(parents=True)
 (dst/'meta').mkdir();shutil.copy2(src/'meta/tasks.parquet',dst/'meta/tasks.parquet')
 tables=[pq.read_table(p) for p in sorted((src/'data').rglob('*.parquet'))];data=pa.concat_tables(tables)
 metadata=[pq.read_table(p) for p in sorted((src/'meta/episodes').rglob('*.parquet'))];schema=metadata[0].schema
 rows=sorted([r for t in metadata for r in t.to_pylist()],key=lambda r:r['episode_index'])
 keys=[k for k,v in info['features'].items() if v['dtype']=='video'];jobs=[];all_stats=[];new_rows=[];mapping=[];offset=0
 for old in rows:
  eid=old['episode_index'];r={k:v for k,v in old.items() if not k.startswith('stats/')}
  t=data.filter(pa.compute.equal(data['episode_index'],eid));assert len(t)==old['length']
  selection=np.arange(0,len(t),2);t=t.take(pa.array(selection));n=len(t)
  for key,values in [('timestamp',np.arange(n)/15),('frame_index',np.arange(n)),('index',np.arange(offset,offset+n))]:
   j=t.schema.get_field_index(key);t=t.set_column(j,t.schema.field(j),pa.array(values,type=t.schema.field(j).type))
  chunk,file=divmod(eid,info['chunks_size'])
  p=dst/info['data_path'].format(chunk_index=chunk,file_index=file);p.parent.mkdir(parents=True,exist_ok=True);pq.write_table(t,p)
  r.update({'length':n,'data/chunk_index':chunk,'data/file_index':file,'dataset_from_index':offset,'dataset_to_index':offset+n,'meta/episodes/chunk_index':chunk,'meta/episodes/file_index':file})
  numeric={k:np.asarray(t[k].to_pylist()) for k in t.column_names if k in info['features']}
  stats=compute_episode_stats(numeric,info['features']);all_stats.append(stats);new_rows.append(r)
  for key in keys:
   pre='videos/'+key+'/'
   assert old[pre+'from_timestamp']==0
   source=src/info['video_path'].format(video_key=key,chunk_index=old[pre+'chunk_index'],file_index=old[pre+'file_index'])
   target=dst/info['video_path'].format(video_key=key,chunk_index=chunk,file_index=file)
   jobs.append((source,target,n,key,stats))
   r.update({pre+'chunk_index':chunk,pre+'file_index':file,pre+'from_timestamp':0.,pre+'to_timestamp':n/15})
  mapping.append({'episode_index':eid,'source_length':old['length'],'length':n,'source_dataset_from_index':old['dataset_from_index'],'dataset_from_index':offset})
  offset+=n
 print('Prepared rows',len(rows),'frames',offset,flush=True)
 def video(job):
  source,target,n,key,stats=job;target.parent.mkdir(parents=True,exist_ok=True)
  # Lossless H264 preserves retained decoded frames while removing odd source frames.
  subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-i',str(source),'-map','0:v:0','-vf',"select=not(mod(n\\,2)),setpts=N/(15*TB)",'-r','15','-an','-c:v','libx264','-crf','0','-preset','fast','-g','2','-pix_fmt','yuv420p','-threads','2',str(target)],check=True)
  sampled=set(sample_indices(n));imgs=[];count=0
  with av.open(str(source)) as before,av.open(str(target)) as after:
   st=after.streams.video[0];assert st.frames==n and st.average_rate==15 and st.start_time==0
   assert abs(float(st.duration*st.time_base)-n/15)<1e-4
   oldframes=(f for i,f in enumerate(before.decode(video=0)) if i%2==0)
   for frame in after.decode(video=0):
    original=next(oldframes);rgb=frame.to_ndarray(format='rgb24')
    assert np.array_equal(rgb,original.to_ndarray(format='rgb24')),(target,count,'pixel mismatch')
    assert abs(float(frame.pts*frame.time_base)-count/15)<1e-4
    if count in sampled:imgs.append(auto_downsample_height_width(rgb.transpose(2,0,1)))
    count+=1
   assert next(oldframes,None) is None and count==n,(target,count,n)
  raw=get_feature_stats(np.stack(imgs),axis=(0,2,3),keepdims=True)
  stats[key]={k:v if k=='count' else np.squeeze(v/255.,axis=0) for k,v in raw.items()}
 with ThreadPoolExecutor(max_workers=4) as pool:
  for i,_ in enumerate(pool.map(video,jobs),1):
   if i%12==0:print('Videos fully decoded and pixel-verified',i,'/',len(jobs),flush=True)
 for r,stats in zip(new_rows,all_stats,strict=True):
  for feat,values in stats.items():
   for stat,value in values.items():r[f'stats/{feat}/{stat}']=value.tolist()
  p=dst/f"meta/episodes/chunk-{r['meta/episodes/chunk_index']:03d}/file-{r['meta/episodes/file_index']:03d}.parquet";p.parent.mkdir(parents=True,exist_ok=True)
  pq.write_table(pa.Table.from_pylist([r],schema=schema),p)
 info['fps']=15;info['total_frames']=offset
 for key in keys:info['features'][key]['info'].update({'video.fps':15,'video.crf':0,'video.preset':'fast'})
 (dst/'meta/info.json').write_text(json.dumps(info,indent=2)+'\n')
 stats=aggregate_stats(all_stats)
 (dst/'meta/stats.json').write_text(json.dumps({k:{s:v.tolist() for s,v in values.items()} for k,values in stats.items()},indent=2)+'\n')
 old_ds=LeRobotDataset('local/wamu-original',root=src,video_backend='pyav');new_ds=LeRobotDataset('local/wamu-15hz',root=dst,video_backend='pyav')
 assert len(new_ds)==offset and new_ds.num_episodes==len(rows)
 for m in mapping:
  for j in [0,m['length']//2,m['length']-1]:
   a=old_ds[m['source_dataset_from_index']+2*j];b=new_ds[m['dataset_from_index']+j]
   for key in keys+['action','observation.state','episode_index','task_index']:assert torch.equal(a[key],b[key]),(m['episode_index'],j,key)
 # Validate all retained numerical rows, dense indices and timing independently.
 check=pa.concat_tables([pq.read_table(p) for p in sorted((dst/'data').rglob('*.parquet'))])
 assert check['index'].to_pylist()==list(range(offset))
 for m in mapping:
  old=data.filter(pa.compute.equal(data['episode_index'],m['episode_index'])).take(pa.array(np.arange(0,m['source_length'],2)))
  new=check.filter(pa.compute.equal(check['episode_index'],m['episode_index']))
  for k in ['action','observation.state','task_index','episode_index']:assert old[k].equals(new[k])
  assert new['frame_index'].to_pylist()==list(range(m['length']))
  assert np.allclose(np.array(new['timestamp']),np.arange(m['length'])/15,atol=1e-5)
 report={'source_fps':30,'fps':15,'source_frames':len(data),'frames':offset,'episodes':len(rows),'selection':'Per episode retain source frame indices 0,2,4,...; odd lengths rounded up','video_encoding':'H264 lossless (CRF 0), yuv420p; every retained decoded RGB frame compared exactly','statistics':'All numeric episode statistics recomputed; RGB statistics resampled from decoded 15-Hz videos using LeRobot sampling/downsampling; global stats reaggregated','validation':'Full video decode and pixel/timestamp/frame-count checks; all state/action rows matched retained source; v3 loader first/middle/last samples for every episode passed','backup':str(backup),'mapping':mapping}
 (dst/'resampling_report.json').write_text(json.dumps(report,indent=2)+'\n')
 (dst/'video_layout.json').write_text(json.dumps({'format':'LeRobot Dataset v3.0','layout':'one video per episode per camera','episodes':len(rows),'videos':len(jobs),'fps':15,'method':report['video_encoding']},indent=2)+'\n')
 # Keep source_report only in backup: its 30-Hz timing/sample metadata no longer describes output.
 backup.parent.mkdir(exist_ok=True);src.rename(backup)
 try:dst.rename(src)
 except BaseException:backup.rename(src);raise
 print('PUBLISHED',src,len(rows),offset,flush=True)

if __name__=='__main__':
 targets=[p.parent.parent for p in sorted((ROOT/'wamu_real').glob('*/meta/info.json')) if json.loads(p.read_text())['fps']==30]
 assert targets,'No 30-Hz datasets found'
 for target in targets:convert(target)
