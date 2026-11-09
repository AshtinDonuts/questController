"""Official v3 readback and native recording lifecycle, entirely offline."""
import json
from pathlib import Path

import h5py
import numpy as np
import pytest

from webxr_teleop.datasets.config import load_config
from webxr_teleop.datasets.export_lerobot import episode_paths, export
from webxr_teleop.datasets.lerobot_recording import NativeRecording
from test_dataset_hdf5 import make_episode


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    monkeypatch.setenv('HF_HUB_OFFLINE', '1')


def config(tmp_path, **updates):
    cfg = dict(dataset_dir='native', task='test', cameras={}, display_data=False,
               format='lerobot_v3', lerobot=dict(repo_id='local/native-test'))
    cfg.update(updates)
    path = tmp_path / 'record.json'
    path.write_text(json.dumps(cfg))
    return load_config(path, mock=True)


@pytest.mark.parametrize('update', [dict(format='v3'), dict(lerobot={}),
    dict(lerobot=dict(repo_id='invalid')), dict(lerobot=dict(repo_id='local/test', fps=True)),
    dict(lerobot=dict(repo_id='local/test', robot_tolerance_s=0)),
    dict(lerobot=dict(repo_id='local/test', camera_tolerance_s=-1)),
    dict(format='hdf5', lerobot=dict(repo_id='local/test'))])
def test_invalid_native_config(tmp_path, update):
    with pytest.raises(ValueError):
        config(tmp_path, **update)


def test_converter_selects_dataset_accepted_or_file_and_refuses_discard(tmp_path):
    saved = make_episode(tmp_path, cameras=False)
    discarded = make_episode(tmp_path, cameras=False, identity=1, status='discarded')
    for source in (tmp_path, saved.parent, saved):
        assert episode_paths(source) == [saved]
    pytest.importorskip('lerobot.configs')
    with pytest.raises(ValueError, match='Expected a saved'):
        export(discarded, tmp_path / 'bad-export', 'local/test')
    assert not (tmp_path / 'bad-export').exists()


def source(tmp_path, cameras=True, sides=('left','right')):
    root = tmp_path / 'source'
    root.mkdir()
    path = make_episode(root, sides=sides, cameras=cameras)
    with h5py.File(path, 'r') as file:
        metadata = json.loads(file['metadata/json'][()])
    return path, metadata


def feed(episode, path):
    with h5py.File(path, 'r') as file:
        for side in episode.metadata['sides']:
            for row in file[f'streams/robot/{side}/observations/json'][:]:
                episode.robot(side, json.loads(row))
        for name, cfg in episode.metadata['config']['cameras'].items():
            for stream in cfg['streams']:
                group = file[f'streams/cameras/{name}/{stream}']
                for i, timestamp in enumerate(group['t_ns'][:]):
                    episode.camera(name, stream, dict(t_ns=int(timestamp), frame_number=i,
                        image=group['image'][i], device_ms=float(group['device_ms'][i])))


def recording(tmp_path, metadata):
    cfg = config(tmp_path)
    cfg['cameras'] = metadata['config']['cameras']
    metadata['config'] = cfg
    root = Path(cfg['dataset_dir'])
    root.mkdir(exist_ok=True)
    return NativeRecording(root, cfg, metadata)


@pytest.mark.parametrize('cameras,sides', [(False, ('left',)), (True, ('left','right'))])
def test_native_matches_conversion_and_resumes(tmp_path, cameras, sides):
    pytest.importorskip('lerobot.configs')
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    path, metadata = source(tmp_path, cameras, sides)
    native = recording(tmp_path, metadata)
    origin = 9_000_000_000_000_000
    end = origin + 1_010_000_000
    attempt = native.episode(native.next_identity(), origin, metadata)
    feed(attempt, path)
    attempt.finish('discarded', end)
    assert not native.destination.exists()
    assert native.next_identity() == 1
    for _ in range(2):
        native = NativeRecording(native.root, native.config, metadata)
        attempt = native.episode(native.next_identity(), origin, metadata)
        feed(attempt, path)
        attempt.finish('saved', end, outcome='success')
    assert native.saved == 2
    assert not list(native.attempts.glob('*.partial.sqlite'))
    assert not list(native.attempts.glob('*.committing'))
    assert not list(native.root.rglob('*.hdf5'))
    converted = tmp_path / 'converted'
    export(path, converted, 'local/converted')
    expected = LeRobotDataset('local/converted', root=converted, video_backend='pyav', depth_output_unit='m')
    actual = LeRobotDataset('local/native-test', root=native.destination, video_backend='pyav', depth_output_unit='m')
    assert actual.meta.info.codebase_version == 'v3.0'
    assert len(actual) == 2 * len(expected)
    for i in (0, len(expected)//2, len(expected)-1):
        for key in ('observation.state', 'action'):
            np.testing.assert_array_equal(actual[i][key], expected[i][key])
            np.testing.assert_array_equal(actual[len(expected)+i][key], expected[i][key])
        if cameras:
            np.testing.assert_allclose(actual[i]['observation.depth.front'], expected[i]['observation.depth.front'], atol=.001)
            assert actual[i]['observation.images.front'].shape == (3,48,64)
    report = json.loads((native.attempts / 'episode_000001.saved.json').read_text())
    assert report['outcome'] == 'success'
    assert report['frames'] == len(expected)
    for key, plan in report['alignment'].items():
        assert all(d >= 0 for d in plan['offset_ns']) if key.endswith('actions') else all(d <= 0 for d in plan['offset_ns'])


def test_pause_trims_cross_channel_tail_and_excludes_gap(tmp_path):
    pytest.importorskip('lerobot.configs')
    path, metadata = source(tmp_path, cameras=False)
    native = recording(tmp_path, metadata)
    origin = 9_000_000_000_000_000
    attempt = native.episode(0, origin, metadata)
    feed(attempt, path)
    attempt.pause(origin + 400_000_000, 'tracking lost')
    attempt.resume(origin + 800_000_000)
    feed(attempt, path)
    attempt.finish('saved', origin + 1_010_000_000)
    report = json.loads((native.attempts / 'episode_000000.saved.json').read_text())
    assert 10 < report['frames'] < 25
    assert [e['kind'] for e in report['events']] == ['start','pause','resume','end']


def test_native_gap_fails_before_dataset_creation(tmp_path):
    pytest.importorskip('lerobot.configs')
    path, metadata = source(tmp_path, cameras=False)
    native = recording(tmp_path, metadata)
    cfg = native.config
    cfg['lerobot']['robot_tolerance_s'] = .00001
    attempt = native.episode(0, 9_000_000_000_000_000, metadata)
    feed(attempt, path)
    try:
        with pytest.raises(ValueError, match='gap exceeds tolerance'):
            attempt.finish('saved', attempt.start_ns + 1_010_000_000)
        assert not native.destination.exists()
        assert attempt.path.exists()
        assert attempt.path.name.endswith('.rejected.sqlite')
        assert native.saved == 0
        assert not list(native.attempts.glob('*.committing'))
        assert NativeRecording(native.root, native.config, metadata).next_identity() == 1
    finally:
        attempt.close_partial()


def test_rejected_save_keeps_recorder_alive_and_next_attempt_can_save(tmp_path):
    import sys
    import time
    from webxr_teleop.datasets.client import RecorderClient
    from test_dataset_recorder import wait
    pytest.importorskip('lerobot.configs')
    _, metadata = source(tmp_path, cameras=False, sides=('left',))
    config(tmp_path)
    meta_path = tmp_path / 'metadata.json'
    meta_path.write_text(json.dumps(metadata))
    client = RecorderClient(tmp_path/'record.json', meta_path, sys.executable, 2)
    try:
        wait(client, lambda c:c.phase=='idle', seconds=15)
        client.request('start', start_ns=time.monotonic_ns())
        wait(client, lambda c:c.phase=='recording')
        # Missing stream coverage rejects this attempt before dataset creation.
        client.request('finish', status='saved', end_ns=time.monotonic_ns())
        wait(client, lambda c:c.phase=='finished' or c.error is not None)
        assert client.error is None
        assert client.status['last_status']=='rejected'
        assert client.status['saved']==0
        assert 'No common valid stream coverage' in client.status['save_rejection']
        assert Path(client.status['path']).name.endswith('.rejected.sqlite')
        assert Path(client.status['path']).exists()
        assert client.process.poll() is None
        client.request('start', start_ns=time.monotonic_ns())
        wait(client, lambda c:c.phase=='recording')
        assert 'save_rejection' not in client.status
        for seq in range(1, 26):
            timestamp=time.monotonic_ns()
            client.sample('robot', side='left', value=dict(t=timestamp/1e9, seq=seq,
                q=[.1]*6+[.5], action=dict(t_ns=timestamp+1000,
                qpos=[.2]*6+[.6], confirmed=True)))
            client.poll()
            time.sleep(.02)
        client.request('finish', status='saved', end_ns=time.monotonic_ns())
        wait(client, lambda c:c.phase=='finished' or c.error is not None, seconds=30)
        assert client.error is None
        assert client.status['last_status']=='saved'
        assert client.status['saved']==1
    finally:
        client.close()


def test_native_resume_refuses_incompatible_rig_and_incomplete_commit(tmp_path):
    pytest.importorskip('lerobot.configs')
    _, metadata = source(tmp_path, cameras=False)
    native = recording(tmp_path, metadata)
    cfg = dict(native.config, lerobot=dict(native.config['lerobot'], fps=20))
    with pytest.raises(ValueError, match='schema, rig'):
        NativeRecording(native.root, cfg, metadata)
    (native.attempts / 'episode_000003.committing').touch()
    with pytest.raises(RuntimeError, match='Interrupted LeRobot commit'):
        NativeRecording(native.root, native.config, metadata)


@pytest.mark.parametrize('full_resolution', [False, True])
def test_native_recorder_process_saves_and_enforces_limit_after_restart(tmp_path, full_resolution):
    import sys
    import time
    from webxr_teleop.datasets.client import RecorderClient
    from test_dataset_recorder import wait
    pytest.importorskip('lerobot.configs')
    camera = dict(backend='synthetic', width=640 if full_resolution else 64,
                  height=480 if full_resolution else 48, fps=30, streams=['rgb','depth'])
    cfg = config(tmp_path, num_episodes=1,
                 cameras={name:dict(camera) for name in (('left_arm','top') if full_resolution else ('front',))})
    config_path = tmp_path / 'record.json'
    # The serialized config is loaded by the actual recorder process.
    path, metadata = source(tmp_path, cameras=False, sides=('left',))
    meta_path = tmp_path / 'metadata.json'
    meta_path.write_text(json.dumps(metadata))
    client = RecorderClient(config_path, meta_path, sys.executable, 1)
    try:
        wait(client, lambda c:c.phase=='idle', seconds=10)
        client.request('start', start_ns=time.monotonic_ns())
        wait(client, lambda c:c.phase=='recording')
        for seq in range(1, 51):
            timestamp = time.monotonic_ns()
            client.sample('robot', side='left', value=dict(t=timestamp/1e9, seq=seq, epoch=1,
                q=[.1]*6+[.5], goal=[.2]*6+[.6], action=dict(t_ns=timestamp+1000,
                qpos=[.2]*6+[.6], confirmed=True)))
            client.poll()
            assert client.error is None
            time.sleep(.02)
        client.request('finish', status='saved', end_ns=time.monotonic_ns())
        wait(client, lambda c:c.phase=='finished' or c.error is not None, seconds=20)
        assert client.error is None
        assert client.status['saved']==1
        assert Path(client.status['path'])==Path(cfg['dataset_dir'])/'lerobot'
    finally:
        client.close()
    client = RecorderClient(config_path, meta_path, sys.executable, 1)
    try:
        wait(client, lambda c:c.phase=='idle', seconds=10)
        assert client.status['saved']==1
        client.request('start', start_ns=time.monotonic_ns())
        wait(client, lambda c:c.pending is None and bool(c.status.get('rejected')))
        assert 'target reached' in client.status['rejected']
    finally:
        client.close()


def test_native_preview_serializes_with_numpy2_compatible_rerun(tmp_path):
    rr = pytest.importorskip('rerun')
    from webxr_teleop.datasets.viewer import blueprint
    rr.init('native-v3-preview-test')
    path = tmp_path / 'preview.rrd'
    try:
        rr.save(path)
        rr.send_blueprint(blueprint({'front':dict(streams=['rgb','depth'])}))
        rr.set_time_seconds('session', 1.)
        rr.log('cameras/front/rgb', rr.Image(np.zeros((48,64,3), dtype=np.uint8)))
        rr.log('cameras/front/depth', rr.DepthImage(np.ones((48,64), dtype=np.uint16), meter=1000))
        rr.log('robot/left/waist/measured', rr.Scalar(.1))
    finally:
        rr.disconnect()
    assert path.stat().st_size > 0


def test_failed_commit_keeps_attempt_and_blocks_future_appends(tmp_path, monkeypatch):
    pytest.importorskip('lerobot.configs')
    from webxr_teleop.datasets import lerobot_recording as module
    from webxr_teleop.datasets.recorder import finish_episode
    path, metadata = source(tmp_path, cameras=False)
    native = recording(tmp_path, metadata)
    attempt = native.episode(0, 9_000_000_000_000_000, metadata)
    feed(attempt, path)
    def fail(*args, **kwargs):
        raise OSError('disk full')
    monkeypatch.setattr(module, 'create_dataset', fail)
    with pytest.raises(OSError, match='disk full'):
        finish_episode(attempt, 'saved', attempt.start_ns + 1_010_000_000)
    assert attempt.closed and attempt.path.exists()
    assert native.saved == 0
    with pytest.raises(RuntimeError, match='Interrupted LeRobot commit'):
        NativeRecording(native.root, native.config, metadata)
