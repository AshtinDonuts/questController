"""Contract tests for the offline native-HDF5 -> ACT training bridge."""
import hashlib
import json

import h5py
import numpy as np
import pytest

from test_dataset_hdf5 import make_episode
from webxr_teleop.datasets.act_bridge import plan, prepare
from webxr_teleop.datasets.train_act import load_bundle, smoke_check, train


def add_pause(path):
    with h5py.File(path, 'r+') as file:
        origin = int(file.attrs['start_ns'])
        times = [origin, origin + 400_000_000, origin + 600_000_000, int(file.attrs['end_ns'])]
        for name in ('t_ns', 'json'):
            file['events/' + name].resize(4, axis=0)
        file['events/t_ns'][:] = times
        file['events/json'][:] = [json.dumps(dict(kind=k)) for k in ('start', 'pause', 'resume', 'end')]


def test_plan_selects_rgb_canonical_sides_and_auto_rate(tmp_path):
    path = make_episode(tmp_path)
    with h5py.File(path, 'r+') as file:
        metadata = json.loads(file['metadata/json'][()])
        metadata['config']['cameras']['front']['fps'] = 15
        file['metadata/json'][()] = json.dumps(metadata)
    p = plan(tmp_path, sides=['right', 'left'])
    assert p['fps'] == 15
    assert p['sides'] == ['left', 'right']
    assert p['cameras'] == ['front']
    assert all('depth' not in k for k in p['episodes'][0]['segments'][0][1])
    assert plan(path, sides=['left'])['sides'] == ['left']


def test_plan_filters_outcomes_and_refuses_invalid_selection(tmp_path):
    path = make_episode(tmp_path)
    for kwargs in ({'sides': ['invalid']}, {'cameras': ['absent']}, {'fps': 0}, {'fps': True},
                   {'outcomes': ['success']}, {'cameras': []}, {'sides': []}):
        with pytest.raises(ValueError):
            plan(tmp_path, **kwargs)
    with pytest.raises(ValueError, match='accepted'):
        plan(make_episode(tmp_path, identity=1, status='discarded'))
    assert plan(path.parent)['episodes'][0]['path'] == path


def test_plan_rejects_mixed_mappings_before_creating_destination(tmp_path):
    make_episode(tmp_path)
    path = make_episode(tmp_path, identity=1)
    with h5py.File(path, 'r+') as file:
        metadata = json.loads(file['metadata/json'][()])
        metadata['profile']['arms']['left']['offset_deg'][0] += 1
        file['metadata/json'][()] = json.dumps(metadata)
    target = tmp_path / 'export'
    with pytest.raises(ValueError, match='Incompatible'):
        prepare(tmp_path, target, 'local/test')
    assert not target.exists()


def test_pause_creates_independent_training_segments(tmp_path):
    path = make_episode(tmp_path)
    add_pause(path)
    segments = plan(tmp_path)['episodes'][0]['segments']
    assert len(segments) == 2
    origin = 9_000_000_000_000_000
    assert segments[0][0][-1] < origin + 400_000_000
    assert segments[1][0][0] >= origin + 600_000_000
    for grid, indices, offsets in segments:
        assert np.all(offsets['streams/robot/left/actions'] >= 0)
        assert np.all(offsets['streams/cameras/front/rgb'] <= 0)


def test_legacy_export_readback_padding_pixels_and_backward(tmp_path, monkeypatch):
    pytest.importorskip('lerobot.common.datasets.lerobot_dataset')
    monkeypatch.setenv('HF_HUB_OFFLINE', '1')
    # Bound CPU thread overhead for the tiny model without changing the environment.
    import torch
    torch.set_num_threads(2)
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    path = make_episode(tmp_path)
    add_pause(path)
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    target = tmp_path / 'bridge'
    report = prepare(tmp_path, target, 'local/test', sides=['left'], chunk_size=5)
    assert report['training_episodes'] == 2
    assert report['joint_names'][0] == 'left.waist'
    assert report['episodes'][0]['source_sha256'] == before
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    assert not (target / 'INCOMPLETE').exists()
    config = load_bundle(target)[2]
    assert set(config['policy']['input_shapes']) == {'observation.state', 'observation.images.front'}
    dataset = LeRobotDataset('local/test', root=target,
                             delta_timestamps=config['training']['delta_timestamps'])
    assert dataset[0]['action'].shape == (5, 7)
    end = int(dataset.episode_data_index['to'][0]) - 1
    tail = dataset[end]
    assert tail['action_is_pad'].tolist() == [False, True, True, True, True]
    torch.testing.assert_close(tail['action'][0], tail['action'][-1])
    assert not torch.equal(tail['action'][0], dataset[end + 1]['action'][0])
    with h5py.File(path, 'r') as file:
        alignment = report['episodes'][0]['segments'][0]['alignment']
        source_index = alignment['streams/cameras/front/rgb']['indices'][0]
        expected = torch.from_numpy(file['streams/cameras/front/rgb/image'][source_index]).permute(2, 0, 1) / 255
        for target_key, kind in (('observation.state', 'observations'), ('action', 'actions')):
            source_key = f'streams/robot/left/{kind}'
            expected_joints = torch.from_numpy(file[source_key + '/qpos'][alignment[source_key]['indices'][0]]).float()
            actual = dataset[0][target_key]
            torch.testing.assert_close(actual[0] if kind == 'actions' else actual, expected_joints)
    torch.testing.assert_close(dataset[0]['observation.images.front'], expected)
    result = smoke_check(target, small_model=True)
    assert result['backward'] and np.isfinite(result['loss'])
    with pytest.raises(FileExistsError):
        prepare(tmp_path, target, 'local/test')
    (target / 'INCOMPLETE').touch()
    with pytest.raises(ValueError, match='incomplete'):
        load_bundle(target)


def test_prepare_validates_repo_id_and_chunk_size(tmp_path):
    for repo in ('../../escape', '/absolute', 'name', 'local/..'):
        with pytest.raises(ValueError, match='repo-id'):
            prepare(tmp_path, tmp_path / 'out', repo)
    with pytest.raises(ValueError, match='chunk_size'):
        prepare(tmp_path, tmp_path / 'out', 'local/test', chunk_size=0)


def test_failed_preparation_remains_quarantined(tmp_path, monkeypatch):
    pytest.importorskip('lerobot.common.datasets.lerobot_dataset')
    import lerobot.common.datasets.compute_stats as statistics
    make_episode(tmp_path)
    def fail(*args, **kwargs):
        raise RuntimeError('Injected statistics failure')
    monkeypatch.setattr(statistics, 'compute_stats', fail)
    target = tmp_path / 'failed'
    with pytest.raises(RuntimeError, match='Injected'):
        prepare(tmp_path, target, 'local/test')
    assert (target / 'INCOMPLETE').is_file()
    with pytest.raises(ValueError, match='incomplete'):
        load_bundle(target)


def test_training_entrypoint_writes_checkpoint(tmp_path, monkeypatch):
    pytest.importorskip('lerobot.common.policies.act.modeling_act')
    monkeypatch.setenv('OMP_NUM_THREADS', '2')
    make_episode(tmp_path, sides=('left',))
    target = tmp_path / 'bundle'
    prepare(tmp_path, target, 'local/checkpoint', chunk_size=5)
    path = target / 'act_config.json'
    config = json.loads(path.read_text())
    config['policy'].update(dim_model=32, n_heads=4, dim_feedforward=64,
                            n_encoder_layers=1, n_decoder_layers=1, n_vae_encoder_layers=1)
    path.write_text(json.dumps(config))
    output = tmp_path / 'training'
    train(target, output, steps=1, batch_size=1, device='cpu')
    assert (output / 'checkpoints/000001/training_state.pth').is_file()
    assert (output / 'checkpoints/000001/pretrained_model/model.safetensors').is_file()
    with pytest.raises(FileExistsError):
        train(target, output, steps=1)
