"""Prepare native Quest recordings for the installed LeRobot v1.6 ACT fork.

Offline only. The recorder and source files remain unchanged. RGB is embedded
losslessly in Arrow, avoiding video codec and external image-path dependencies.
"""
import argparse
import hashlib
import json
import re
import tempfile
from pathlib import Path

import h5py
import numpy as np

from .alignment import aligned_segments
from .hdf5 import validate
from ..hardware.core import JOINTS


def plan(source, *, fps=None, sides=None, cameras=None, outcomes=None,
         robot_tolerance_s=.04, camera_tolerance_s=None):
    """Validate all selected sources before allocating a training destination."""
    source = Path(source).resolve()
    if source.is_file():
        paths = [source]
    else:
        accepted = source if source.name == 'accepted' else source / 'accepted'
        paths = sorted(accepted.glob('episode_*.hdf5'))
    if not paths or any(p.parent.name != 'accepted' or p.suffix != '.hdf5' for p in paths):
        raise ValueError('Select saved .hdf5 episodes from an accepted directory')
    if outcomes is not None and not set(outcomes) <= {'success', 'failure', 'unknown'}:
        raise ValueError('Invalid outcome filter')
    plans, reference, chosen_fps = [], None, fps
    for path in paths:
        with h5py.File(path, 'r') as file:
            if outcomes is not None and file.attrs.get('outcome') not in outcomes:
                continue
        validate(path)
        with h5py.File(path, 'r') as file:
            metadata = json.loads(file['metadata/json'][()])
            selected_sides = list(sides) if sides is not None else metadata['sides']
            if (not selected_sides or len(set(selected_sides)) != len(selected_sides)
                    or not set(selected_sides) <= set(metadata['sides'])):
                raise ValueError(f'{path}: selected arms must be unique recorded sides')
            selected_sides = [s for s in ('left', 'right') if s in selected_sides]
            if not selected_sides:
                raise ValueError('Expected left and/or right arm')
            camera_cfg = metadata['config']['cameras']
            selected_cameras = sorted(camera_cfg) if cameras is None else list(cameras)
            if cameras is None:
                selected_cameras = [c for c in selected_cameras if 'rgb' in camera_cfg[c]['streams']]
            if (not selected_cameras or len(set(selected_cameras)) != len(selected_cameras)
                    or any(c not in camera_cfg or 'rgb' not in camera_cfg[c]['streams']
                           for c in selected_cameras)):
                raise ValueError(f'{path}: ACT requires unique recorded RGB cameras')
            shapes = {(camera_cfg[c]['height'], camera_cfg[c]['width']) for c in selected_cameras}
            if len(shapes) != 1:
                raise ValueError('ACT camera images must have the same shape; select compatible cameras')
            rates = [camera_cfg[c]['fps'] for c in selected_cameras]
            current_fps = min(rates) if fps is None else fps
            if chosen_fps is None:
                chosen_fps = current_fps
            if chosen_fps != current_fps:
                raise ValueError('Camera rates differ across episodes; export compatible sessions separately')
            signature = dict(mode=metadata['mode'], sides=selected_sides,
                cameras={c: {k: camera_cfg[c][k] for k in ('width', 'height', 'fps')}
                         for c in selected_cameras},
                arms={s: {k: metadata['profile']['arms'][s][k]
                          for k in ('sign', 'offset_deg', 'base_position', 'base_quaternion_wxyz')}
                      for s in selected_sides},
                calibrations={s: metadata.get('calibrations', {}).get(s) for s in selected_sides})
            if reference is not None and signature != reference:
                raise ValueError('Incompatible recording modes, arms, mappings, calibrations or cameras; export separately')
            reference = signature
            segments = aligned_segments(file, chosen_fps, robot_tolerance_s, camera_tolerance_s,
                sides=selected_sides, cameras=selected_cameras, include_depth=False)
            plans.append(dict(path=path, metadata=metadata, segments=segments,
                              outcome=str(file.attrs['outcome']), uuid=str(file.attrs['episode_uuid'])))
    if not plans:
        raise ValueError('No accepted episodes match the selection')
    return dict(fps=chosen_fps, sides=selected_sides, cameras=selected_cameras,
                signature=reference, episodes=plans)


def _legacy_api():
    try:
        from lerobot.common.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
    except ImportError as exc:
        raise RuntimeError('Use the existing LeRobot v1.6 fork environment for this bridge; '
                           'the modern v3 exporter is a separate command') from exc
    if CODEBASE_VERSION != 'v1.6':
        raise RuntimeError(f'Expected LeRobot dataset API v1.6, got {CODEBASE_VERSION}')
    return LeRobotDataset


def _features(prepared):
    from datasets import Features, Image, Sequence, Value
    size = len(prepared['sides']) * 7
    return Features({
        'observation.state': Sequence(Value('float32'), length=size),
        'action': Sequence(Value('float32'), length=size),
        **{f'observation.images.{c}': Image() for c in prepared['cameras']},
        'episode_index': Value('int64'), 'frame_index': Value('int64'),
        'index': Value('int64'), 'timestamp': Value('float32'),
        'next.done': Value('bool'),
    })


def _frames(prepared):
    from PIL import Image
    episode_index, index = 0, 0
    for episode in prepared['episodes']:
        with h5py.File(episode['path'], 'r') as file:
            for grid, indices, _ in episode['segments']:
                for i in range(len(grid)):
                    frame = dict(episode_index=episode_index, frame_index=i, index=index,
                                 timestamp=i / prepared['fps'])
                    frame['next.done'] = i == len(grid) - 1
                    for target, kind in (('observation.state', 'observations'), ('action', 'actions')):
                        frame[target] = np.concatenate([
                            file[f'streams/robot/{s}/{kind}/qpos'][indices[f'streams/robot/{s}/{kind}'][i]]
                            for s in prepared['sides']]).astype(np.float32)
                    for camera in prepared['cameras']:
                        key = f'streams/cameras/{camera}/rgb'
                        image = file[key + '/image'][indices[key][i]]
                        if image.dtype != np.uint8:
                            raise ValueError(f'{key}: RGB must be uint8')
                        frame[f'observation.images.{camera}'] = Image.fromarray(image)
                    yield frame
                    index += 1
                episode_index += 1


def _training_config(prepared, repo_id, chunk_size):
    from dataclasses import asdict
    import lerobot
    from omegaconf import OmegaConf
    from lerobot.common.policies.act.configuration_act import ACTConfig
    default_path = Path(lerobot.__file__).parent / 'configs' / 'default.yaml'
    cfg = OmegaConf.load(default_path)
    del cfg['defaults']
    del cfg['hydra']
    shapes = {'observation.state': [len(prepared['sides']) * 7]}
    for camera in prepared['cameras']:
        c = prepared['signature']['cameras'][camera]
        shapes[f'observation.images.{camera}'] = [3, c['height'], c['width']]
    policy = ACTConfig(input_shapes=shapes, output_shapes={'action': shapes['observation.state']},
        input_normalization_modes={k: 'mean_std' for k in shapes},
        chunk_size=chunk_size, n_action_steps=chunk_size, pretrained_backbone_weights=None)
    cfg.policy = dict(name='act', **asdict(policy))
    cfg.dataset_repo_id = repo_id
    cfg.fps = prepared['fps']
    cfg.seed = 1000
    cfg.env = dict(name='dora', task='quest_native_offline',
                   state_dim=len(prepared['sides']) * 7, action_dim=len(prepared['sides']) * 7)
    cfg.training = OmegaConf.merge(cfg.training, dict(
        offline_steps=100000, online_steps=0, eval_freq=0, batch_size=8,
        save_freq=20000, lr=1e-5, lr_backbone=1e-5, weight_decay=1e-4,
        grad_clip_norm=10, num_workers=0,
        delta_timestamps={'action': [i / prepared['fps'] for i in range(chunk_size)]}))
    cfg.wandb.enable = False
    return OmegaConf.to_container(cfg, resolve=True)


def prepare(source, destination, repo_id, *, chunk_size=100, **selection):
    """Write a new self-contained bundle and verify it with the fork's loader."""
    if not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_-]+', repo_id):
        raise ValueError('repo-id must be namespace/name using letters, digits, underscores or hyphens')
    if isinstance(chunk_size, bool) or not isinstance(chunk_size, int) or chunk_size <= 0:
        raise ValueError('chunk_size must be a positive integer')
    destination = Path(destination).resolve()
    if destination.exists():
        raise FileExistsError('Destination must be new; existing data is never overwritten')
    prepared = plan(source, **selection)
    LeRobotDataset = _legacy_api()
    from datasets import Dataset
    from lerobot.common.datasets.compute_stats import compute_stats
    from lerobot.common.datasets.utils import flatten_dict, hf_transform_to_torch
    from safetensors.torch import save_file
    import torch

    config = _training_config(prepared, repo_id, chunk_size)
    destination.mkdir(parents=True, exist_ok=False)
    incomplete = destination / 'INCOMPLETE'
    incomplete.write_text('Preparation has not passed loader verification. Do not train.\n')
    dataset_dir = destination / repo_id
    dataset_dir.mkdir(parents=True)
    with tempfile.TemporaryDirectory(prefix='quest-act-', dir=destination) as cache:
        hf_dataset = Dataset.from_generator(_frames, gen_kwargs={'prepared': prepared},
            features=_features(prepared), cache_dir=cache, keep_in_memory=False,
            writer_batch_size=8)
        hf_dataset.save_to_disk(str(dataset_dir / 'train'))
        hf_dataset.set_transform(hf_transform_to_torch)
        lengths = [len(s[0]) for e in prepared['episodes'] for s in e['segments']]
        stops = np.cumsum(lengths)
        episode_data_index = {'from': torch.tensor(np.r_[0, stops[:-1]]), 'to': torch.tensor(stops)}
        info = dict(codebase_version='v1.6', fps=prepared['fps'], video=False,
                    robot_type='quest_viperx')
        dataset = LeRobotDataset.from_preloaded(hf_dataset=hf_dataset,
            episode_data_index=episode_data_index, info=info)
        stats = compute_stats(dataset, batch_size=8, num_workers=0)
        meta = dataset_dir / 'meta_data'
        meta.mkdir()
        (meta / 'info.json').write_text(json.dumps(info, indent=2) + '\n')
        save_file(flatten_dict(stats), str(meta / 'stats.safetensors'))
        save_file(episode_data_index, str(meta / 'episode_data_index.safetensors'))

    loaded = LeRobotDataset(repo_id, root=destination,
                           delta_timestamps=config['training']['delta_timestamps'])
    if len(loaded) != sum(lengths):
        raise RuntimeError('Loader readback length differs')
    for start, stop in zip(episode_data_index['from'], episode_data_index['to']):
        for index in (int(start), int(stop) - 1):
            item = loaded[index]
            if item['action'].shape != (chunk_size, len(prepared['sides']) * 7):
                raise RuntimeError('Loader readback action shape differs')
            if not all(torch.isfinite(v).all() for v in item.values() if isinstance(v, torch.Tensor)):
                raise RuntimeError('Nonfinite loader sample')

    names = [f'{s}.{j}' for s in prepared['sides'] for j in JOINTS + ('gripper',)]
    warnings = []
    constant = [names[i] for i, std in enumerate(stats['action']['std']) if float(std) < 1e-6]
    if constant:
        warnings.append('Constant action dimensions: ' + ', '.join(constant))
    if any(e['outcome'] != 'success' for e in prepared['episodes']):
        warnings.append('Some selected outcomes are unknown or failure; accepted does not certify task success')
    if len({e['metadata']['config']['task'] for e in prepared['episodes']}) > 1:
        warnings.append('Multiple task labels selected; this ACT configuration is not language conditioned')
    rates = [c['fps'] for c in prepared['signature']['cameras'].values()]
    if prepared['fps'] > min(rates):
        warnings.append('Output FPS exceeds a selected camera rate; aligned RGB frames will be reused')
    if len(prepared['episodes']) < 10:
        warnings.append('Very few source demonstrations; use for pipeline checks and collect more task coverage')
    report = dict(schema='quest_act_bridge.v1', repo_id=repo_id, fps=prepared['fps'],
        frames=len(loaded), training_episodes=len(lengths), joint_names=names,
        cameras=prepared['cameras'], signature=prepared['signature'], warnings=warnings,
        units=dict(joints='rad', gripper='calibrated aperture: 0 closed, 1 open'),
        alignment='Preceding measured states/RGB; following confirmed cycle targets; host reception times',
        pause_policy='Each active segment is a separate training episode', episodes=[])
    for episode in prepared['episodes']:
        with episode['path'].open('rb') as f:
            digest = hashlib.sha256()
            for block in iter(lambda: f.read(1024 * 1024), b''):
                digest.update(block)
        report['episodes'].append(dict(source=str(episode['path']), source_sha256=digest.hexdigest(),
            source_uuid=episode['uuid'], outcome=episode['outcome'], metadata=episode['metadata'],
            segments=[dict(t_ns=g.tolist(), alignment={k: dict(indices=v.tolist(),
                offset_ns=o[k].tolist(), reused_samples=len(v) - len(np.unique(v)))
                for k, v in ix.items()}) for g, ix, o in episode['segments']]))
    (destination / 'act_config.json').write_text(json.dumps(config, indent=2) + '\n')
    (destination / 'source_report.json').write_text(json.dumps(report, indent=2) + '\n')
    incomplete.unlink()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--repo-id', required=True)
    parser.add_argument('--fps', type=int, help='Default: slowest selected RGB camera rate')
    parser.add_argument('--sides', nargs='+', choices=['left', 'right'])
    parser.add_argument('--cameras', nargs='+')
    parser.add_argument('--outcomes', nargs='+', choices=['success', 'failure', 'unknown'])
    parser.add_argument('--chunk-size', type=int, default=100)
    parser.add_argument('--robot-tolerance-s', type=float, default=.04)
    parser.add_argument('--camera-tolerance-s', type=float)
    args = parser.parse_args()
    report = prepare(**vars(args))
    print(json.dumps({k: report[k] for k in ('frames', 'training_episodes', 'fps', 'warnings')}, indent=2))


if __name__ == '__main__':
    main()
