import json
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time
import pytest

ROOT=Path(__file__).resolve().parents[1]

from webxr_teleop.hardware.ipc import pair,send,receive


def recorder_child(control,data,root,scenario,pause,parent):
    from webxr_teleop.datasets import recorder
    from webxr_teleop.datasets.cameras import Camera
    from webxr_teleop.datasets.hdf5 import Episode
    cfg=dict(dataset_dir=str(root),task='review',display_data=False,queue_frames=8,
             flush_interval_s=1,cameras={'front':dict(backend='synthetic',width=8,height=8,
             fps=30,streams=['rgb'],max_gap_s=.2)})
    if scenario=='slow_finish':
        original=Episode.finish
        def slow(self,*args,**kwargs):
            time.sleep(.6)
            return original(self,*args,**kwargs)
        Episode.finish=slow
    else:
        class PausableCamera(Camera):
            def _synthetic(self):
                import numpy as np
                self.metadata={'mock':True}
                i=0
                while not self.stop.is_set():
                    if not pause.is_set():
                        self.emit('rgb',np.zeros((8,8,3),dtype=np.uint8),i)
                        i+=1
                    self.stop.wait(1/30)
        recorder.Camera=PausableCamera
    try:
        recorder.run(cfg,dict(sides=['left'],mode='mock'),control,data,parent)
    except RuntimeError:
        pass


def wait_status(sock,predicate,timeout=4):
    deadline=time.monotonic()+timeout
    latest=None
    while time.monotonic()<deadline:
        for status in receive(sock):
            latest=status
            if predicate(status):return status
        time.sleep(.005)
    raise AssertionError(latest)


def start_child(tmp_path,scenario):
    control,child_control=pair();data,child_data=pair()
    ctx=mp.get_context('spawn');pause=ctx.Event()
    proc=ctx.Process(target=recorder_child,args=(child_control,child_data,tmp_path,scenario,pause,os.getpid()))
    proc.start();child_control.close();child_data.close()
    wait_status(control,lambda s:s['phase']=='idle')
    return proc,control,data,pause


def cleanup(proc,control,data):
    send(control,dict(op='shutdown',request_id='shutdown',through_seq=0))
    proc.join(3)
    if proc.is_alive():proc.terminate();proc.join(2)
    control.close();data.close()


def test_slow_finish_preserves_camera_and_heartbeats(tmp_path):
    proc,control,data,pause=start_child(tmp_path,'slow_finish')
    try:
        send(control,dict(op='start',request_id='start',start_ns=time.monotonic_ns(),through_seq=0))
        wait_status(control,lambda s:s['phase']=='recording')
        for seq in range(1,16):
            stamp=time.monotonic_ns()
            row=dict(t=stamp/1e9,seq=seq,epoch=1,q=[0.]*7,goal=[0.]*7,
                     action=dict(t_ns=stamp+1000,qpos=[0.]*7,confirmed=True))
            assert send(data,dict(seq=seq,kind='robot',side='left',value=row))
            time.sleep(.02)
        send(control,dict(op='finish',request_id='save',status='saved',end_ns=time.monotonic_ns(),through_seq=15))
        finished=wait_status(control,lambda s:s['phase']=='finished')
        assert finished['saved']==1
        time.sleep(.1)
        assert proc.is_alive()
        assert wait_status(control,lambda s:s['cameras']['front']['healthy'])['error'] is None
        assert len(list((tmp_path/'accepted').glob('*.hdf5')))==1
        assert proc.is_alive()
    finally:cleanup(proc,control,data)


def test_stale_camera_start_can_retry(tmp_path):
    proc,control,data,pause=start_child(tmp_path,'stale')
    try:
        pause.set()
        stale=wait_status(control,lambda s:not s['cameras']['front']['healthy'])
        assert stale['phase']=='idle'
        send(control,dict(op='start',request_id='start',start_ns=time.monotonic_ns(),through_seq=0))
        rejected=wait_status(control,lambda s:s.get('rejected'))
        assert rejected['phase']=='idle' and rejected['error'] is None
        pause.clear()
        wait_status(control,lambda s:s['cameras']['front']['healthy'])
        send(control,dict(op='start',request_id='retry',start_ns=time.monotonic_ns(),through_seq=0))
        wait_status(control,lambda s:s['phase']=='recording')
        assert proc.is_alive()
    finally:cleanup(proc,control,data)


def test_export_rejects_mixed_mock_and_live(tmp_path):
    import h5py
    from test_dataset_hdf5 import make_episode
    from webxr_teleop.datasets.export_lerobot import export
    source=tmp_path/'native';source.mkdir()
    make_episode(source,cameras=False,identity=0)
    live=make_episode(source,cameras=False,identity=1)
    with h5py.File(live,'r+') as file:
        metadata=json.loads(file['metadata/json'][()]);metadata['mode']='live'
        file['metadata/json'][()]=json.dumps(metadata)
    with pytest.raises(ValueError,match='differ'):
        export(source,tmp_path/'export','local/review-mixed')
    assert not (tmp_path/'export').exists()


def test_old_save_command_remains_idempotent_after_256_requests(tmp_path):
    import asyncio
    from test_dataset_supervisor import supervisor,exercise
    with supervisor(tmp_path) as (_,port,control):
        async def scenario(state,wait,align):
            await asyncio.to_thread(control,'record-save','original-save')
            await wait(lambda s:s['recording']['phase']=='finished')
            for i in range(256):
                await asyncio.to_thread(control,'record-start',f'intervening-{i}')
            await asyncio.to_thread(control,'record-next','next-episode')
            await wait(lambda s:s['state']=='READY')
            await align()
            await asyncio.to_thread(control,'record-start','second-start')
            await wait(lambda s:s['recording']['phase']=='recording')
            await asyncio.sleep(.2)
            # This ID already saved the FIRST episode and must be a no-op now.
            await asyncio.to_thread(control,'record-save','original-save')
            await asyncio.sleep(.25)
            active=await asyncio.to_thread(control)
            assert active['recording']['phase']=='recording'
            assert active['recording']['saved']==1
            await asyncio.to_thread(control,'record-discard','cleanup')
            await wait(lambda s:s['recording']['phase']=='finished')
        asyncio.run(exercise(port,control,scenario))


def test_camera_failure_reports_frozen_boundary_and_aborts(tmp_path):
    proc,control,data,pause=start_child(tmp_path,'stale')
    try:
        send(control,dict(op='start',request_id='start',start_ns=time.monotonic_ns(),through_seq=0))
        started=wait_status(control,lambda s:s['phase']=='recording')
        time.sleep(.1)
        pause.set()
        failed=wait_status(control,lambda s:s['phase']=='failed')
        assert 'camera stream became stale' in failed['error']
        assert started['start_ns']<failed['end_ns']<=time.monotonic_ns()
        proc.join(3)
        assert not proc.is_alive()
        import h5py
        files=list((tmp_path/'aborted').glob('*.hdf5'))
        assert len(files)==1
        with h5py.File(files[0],'r') as f:
            assert f.attrs['end_ns']==failed['end_ns']
            assert f.attrs['status']=='aborted'
        assert not list((tmp_path/'accepted').glob('*.hdf5'))
    finally:cleanup(proc,control,data)
