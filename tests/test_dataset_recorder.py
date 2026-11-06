"""Acquisition failure isolation, resume and request idempotency."""
import json
from pathlib import Path
import time

import pytest

from webxr_teleop.datasets.cameras import Camera
from webxr_teleop.datasets.client import RecorderClient
from webxr_teleop.datasets.hdf5 import Episode
from test_dataset_hdf5 import make_episode

ROOT=Path(__file__).resolve().parents[1]
PYTHON=Path('/home/King/miniconda3/envs/lerobot/bin/python')


def wait(client,predicate,seconds=5):
    deadline=time.monotonic()+seconds
    while True:
        client.poll()
        if predicate(client):return
        assert time.monotonic()<deadline,(client.phase,client.status,client.error)
        time.sleep(.01)


def client_files(tmp_path,cameras=None):
    config=dict(dataset_dir=str(tmp_path/'dataset'),task='test',cameras=cameras or {},display_data=False)
    path=tmp_path/'config.json';path.write_text(json.dumps(config))
    meta=tmp_path/'metadata.json'
    meta.write_text(json.dumps(dict(sides=['left'],mode='mock',
        profile=json.loads((ROOT/'webxr_teleop/hardware/example_profile.json').read_text()))))
    return path,meta


def test_native_camera_queue_overflow_is_reported_and_memory_is_bounded():
    camera=Camera('test',dict(backend='synthetic',width=4,height=4,fps=200,streams=['rgb']),1)
    try:
        deadline=time.monotonic()+1
        while not camera.error and time.monotonic()<deadline:time.sleep(.005)
        assert 'queue overflow' in camera.error
        assert camera.frames.qsize()==1
    finally:camera.close()


def test_camera_health_uses_clock_after_frame_snapshot(monkeypatch):
    camera=Camera.__new__(Camera)
    camera.config=dict(streams=['rgb'],max_gap_s=.2)
    camera.error=None
    camera.latest={'rgb':1_010_000_000}
    monkeypatch.setattr('webxr_teleop.datasets.cameras.time.monotonic_ns',lambda:1_020_000_000)
    assert camera.healthy(1_000_000_000)
    camera.latest={'rgb':800_000_000}
    assert not camera.healthy(1_000_000_000)
    camera.latest={}
    assert not camera.healthy(1_000_000_000)
    camera.latest={'rgb':1_010_000_000};camera.error='capture failed'
    assert not camera.healthy(1_000_000_000)


def test_fsync_failure_never_publishes_accepted_episode(tmp_path,monkeypatch):
    # Fail the real commit operation after a complete episode has been written.
    monkeypatch.setattr('webxr_teleop.datasets.hdf5.os.fsync',lambda _:(_ for _ in ()).throw(OSError('disk full')))
    with pytest.raises(OSError,match='disk full'):make_episode(tmp_path)
    assert not list((tmp_path/'accepted').glob('*.hdf5'))
    assert len(list(tmp_path.glob('*.partial')))==1


def test_recorder_resume_quarantines_partial_and_deduplicates_start(tmp_path):
    if not PYTHON.exists():pytest.skip('Recording interpreter required')
    config,metadata=client_files(tmp_path)
    root=tmp_path/'dataset';root.mkdir()
    partial=root/'episode_000004.partial';partial.write_bytes(b'Interrupted source, intentionally unreadable')
    client=RecorderClient(config,metadata,str(PYTHON),1)
    try:
        wait(client,lambda c:c.phase=='idle')
        client.request('start',start_ns=time.monotonic_ns())
        request=client.pending.copy()
        wait(client,lambda c:c.phase=='recording')
        from webxr_teleop.hardware.ipc import send
        send(client.control,request)
        time.sleep(.1);client.poll()
        assert client.phase=='recording'
        client.abort('test interruption')
        first=client.pending['request_id']
        client.abort('repeated stop must keep the same abort request')
        assert client.pending['request_id']==first
        wait(client,lambda c:c.phase=='finished')
        assert client.status['episode']==5
    finally:client.close()
    assert partial.read_bytes()==b'Interrupted source, intentionally unreadable'
    assert (root/'aborted/episode_000005.hdf5').exists()


def test_second_recorder_cannot_own_same_dataset(tmp_path):
    if not PYTHON.exists():pytest.skip('Recording interpreter required')
    config,metadata=client_files(tmp_path)
    first=RecorderClient(config,metadata,str(PYTHON),1)
    second=None
    try:
        wait(first,lambda c:c.phase=='idle')
        second=RecorderClient(config,metadata,str(PYTHON),1)
        wait(second,lambda c:c.error is not None)
        assert second.phase=='failed'
        first.poll();assert not first.error
    finally:
        if second:second.close()
        first.close()


def test_saved_episode_limit_survives_restart_and_excludes_discards(tmp_path):
    if not PYTHON.exists():pytest.skip('Recording interpreter required')
    config,metadata=client_files(tmp_path)
    cfg=json.loads(config.read_text());cfg['num_episodes']=1
    config.write_text(json.dumps(cfg))
    root=tmp_path/'dataset';root.mkdir()
    make_episode(root,identity=0,status='discarded')
    client=RecorderClient(config,metadata,str(PYTHON),1)
    try:
        wait(client,lambda c:c.phase=='idle')
        assert client.status['saved']==0
        client.request('start',start_ns=time.monotonic_ns())
        wait(client,lambda c:c.phase=='recording')
        client.abort('test')
        wait(client,lambda c:c.phase=='finished')
    finally:client.close()
    make_episode(root,identity=2)
    client=RecorderClient(config,metadata,str(PYTHON),1)
    try:
        wait(client,lambda c:c.phase=='idle')
        client.request('start',start_ns=time.monotonic_ns())
        wait(client,lambda c:c.pending is None and bool(c.status.get('rejected')))
        assert 'target reached' in client.status['rejected']
        assert client.phase=='idle'
        assert client.status['saved']==1
        assert not list(root.glob('*.partial'))
    finally:client.close()
