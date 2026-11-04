import json

import h5py
import numpy as np
import pytest

from webxr_teleop.datasets import replay


def test_replay_preserves_independent_timing_pixels_and_depth(tmp_path, monkeypatch):
    path = tmp_path / 'episode.hdf5'
    origin = 9_000_000_000_000_000
    with h5py.File(path, 'w') as file:
        file.attrs.update(schema=replay.SCHEMA, status='saved', start_ns=origin, end_ns=origin+1000)
        file['metadata/json'] = json.dumps(dict(config=dict(cameras={}), cameras=dict(front=dict(depth_scale_m=.001))))
        for stream, offset in [('observations', 12), ('actions', 19)]:
            file[f'streams/robot/left/{stream}/t_ns'] = [origin+offset, origin+offset+100]
            file[f'streams/robot/left/{stream}/qpos'] = [[1., 2., 3., 4., 5., 6., .7]] * 2
        file['streams/cameras/front/depth/t_ns'] = [origin+35]
        file['streams/cameras/front/depth/image'] = np.array([[[123]]], dtype=np.uint16)
        for camera in ('front', 'top'):
            file[f'streams/cameras/{camera}/rgb/t_ns'] = [origin+30, origin+130]
            file[f'streams/cameras/{camera}/rgb/image'] = np.zeros((2, 1, 1, 3), dtype=np.uint8)
    calls = []

    class Sink:
        time = None

        def set_time_nanos(self, timeline, nanos):
            assert timeline == 'episode'
            self.time = nanos

        def log(self, path, value, **kwargs):
            calls.append((self.time, path, value))

        def send_blueprint(self, value):
            pass

        def TextDocument(self, *args, **kwargs):
            return args

        def Scalar(self, value):
            return value

        def DepthImage(self, frame, meter):
            return (frame, meter)

        def Image(self, frame):
            return frame

    monkeypatch.setattr(replay, 'blueprint', lambda cameras: None)
    before = path.read_bytes()
    replay.log_episode(path, Sink())
    assert (12, 'robot/left/waist/measured', 1.) in calls
    assert (19, 'robot/left/gripper/command', .7) in calls
    times = [time for time, _, _ in calls if time is not None]
    assert times == sorted(times)
    assert [(time, name) for time, name, _ in calls if name.endswith('/rgb')] == [
        (30, 'cameras/front/rgb'), (30, 'cameras/top/rgb'),
        (130, 'cameras/front/rgb'), (130, 'cameras/top/rgb')]
    time, name, (frame, meter) = next(call for call in calls if call[1].endswith('/depth'))
    assert (time, name, meter) == (35, 'cameras/front/depth', 1000)
    np.testing.assert_array_equal(frame, [[123]])
    assert path.read_bytes() == before


def test_discovery_only_selects_accepted_and_rejects_partial(tmp_path):
    accepted = tmp_path / 'accepted'
    accepted.mkdir()
    saved = accepted / 'episode_000001.hdf5'
    saved.touch()
    (tmp_path / 'episode_000002.partial').touch()
    assert replay.episodes(tmp_path) == [saved]
    assert replay.episodes(accepted) == [saved]
    assert replay.episodes(saved) == [saved]
    partial = tmp_path / 'unfinished.partial'
    with h5py.File(partial, 'w') as file:
        file.attrs['schema'] = replay.SCHEMA
    with pytest.raises(ValueError, match='completed'):
        replay.summary(partial)
