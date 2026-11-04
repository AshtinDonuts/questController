"""Offline native HDF5 -> LeRobot v3 export; never opens robot devices."""
import argparse
import hashlib
from importlib.metadata import version
import json
from pathlib import Path

import h5py
import numpy as np

from .hdf5 import validate
from ..hardware.core import JOINTS


def alignment(file, fps=30, robot_tolerance_s=.04, camera_tolerance_s=None):
    """Causal observations, followed by each arm's next confirmed cycle target."""
    if isinstance(fps, bool) or not isinstance(fps, int) or fps <= 0:
        raise ValueError('fps must be a positive integer')
    if not np.isfinite(robot_tolerance_s) or robot_tolerance_s <= 0:
        raise ValueError('robot tolerance must be finite and positive')
    metadata = json.loads(file['metadata/json'][()])
    streams = {}
    for side in metadata['sides']:
        for name in ('observations', 'actions'):
            key = f'streams/robot/{side}/{name}'
            streams[key] = (file[key]['t_ns'][:], name == 'actions', robot_tolerance_s)
    for name, cfg in metadata['config']['cameras'].items():
        for stream in cfg['streams']:
            key = f'streams/cameras/{name}/{stream}'
            tolerance = 2/cfg['fps'] if camera_tolerance_s is None else camera_tolerance_s
            if not np.isfinite(tolerance) or tolerance <= 0:
                raise ValueError('camera tolerance must be finite and positive')
            streams[key] = (file[key]['t_ns'][:], False, tolerance)
    begin = max(int(t[0]) for t, _, _ in streams.values())
    end = min(int(t[-1]) for t, _, _ in streams.values())
    if end <= begin:
        raise ValueError('No common valid stream coverage')
    # Relative arithmetic avoids precision loss from large host-clock origins.
    count = int((end-begin)*fps//1_000_000_000)+1
    grid = begin+np.rint(np.arange(count)*1e9/fps).astype(np.int64)
    grid = grid[grid <= end]
    indices = {}
    offsets = {}
    for key, (times, following, tolerance) in streams.items():
        selected = np.searchsorted(times, grid, side='left' if following else 'right')
        if not following:
            selected -= 1
        if np.any(selected < 0) or np.any(selected >= len(times)):
            raise ValueError(f'Uncovered timeline: {key}')
        delta = times[selected]-grid
        if np.any(np.abs(delta) > tolerance*1e9):
            raise ValueError(f'Alignment gap exceeds tolerance: {key} ({np.max(np.abs(delta))/1e6:.1f} ms)')
        indices[key] = selected
        offsets[key] = delta
    return grid, indices, offsets


def signature(metadata):
    return dict(mode=metadata['mode'],sides=metadata['sides'], cameras={name:{k:cfg[k] for k in ('width','height','streams')}
                for name,cfg in metadata['config']['cameras'].items()},
                arms={side:{k:metadata['profile']['arms'][side][k]
                      for k in ('sign','offset_deg','base_position','base_quaternion_wxyz')}
                      for side in metadata['sides']})


def export(source, destination, repo_id, *, fps=30, robot_tolerance_s=.04, camera_tolerance_s=None):
    if version('lerobot') != '0.6.1':
        raise RuntimeError('Use the isolated requirements-dataset-export.txt environment (LeRobot 0.6.1)')
    from lerobot.configs import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists():
        raise FileExistsError('Export destination must be new; source recordings are immutable')
    paths = sorted((source/'accepted').glob('episode_*.hdf5'))
    if not paths:
        raise ValueError('No accepted source episodes')
    reference = None
    plans = []
    # Validate every source/alignment before creating any export output.
    for path in paths:
        validate(path)
        with h5py.File(path, 'r') as file:
            metadata = json.loads(file['metadata/json'][()])
            current = signature(metadata)
            if reference is not None and current != reference:
                raise ValueError('Source recording modes, arm/camera schemas or rig mappings differ; export compatible sessions separately')
            reference = current
            grid, indices, offsets = alignment(file, fps, robot_tolerance_s, camera_tolerance_s)
            plans.append((path, metadata, grid, indices, offsets))
    metadata = plans[0][1]
    names = [f'{side}.{joint}' for side in metadata['sides'] for joint in JOINTS+('gripper',)]
    features = {key:dict(dtype='float32', shape=(len(names),), names=names)
                for key in ('observation.state','action')}
    for name, cfg in metadata['config']['cameras'].items():
        for stream in cfg['streams']:
            key = f'observation.images.{name}' if stream == 'rgb' else f'observation.depth.{name}'
            features[key] = dict(dtype='video', shape=(cfg['height'], cfg['width'], 3 if stream == 'rgb' else 1),
                                 names=['height','width','channels'])
            if stream == 'depth':
                features[key]['info'] = dict(is_depth_map=True, depth_unit='m')
    dataset = LeRobotDataset.create(repo_id=repo_id, root=destination, fps=fps,
        robot_type='quest_viperx', features=features, use_videos=True, video_backend='pyav',
        rgb_encoder=RGBEncoderConfig(vcodec='h264', pix_fmt='yuv420p', crf=18, preset='fast'),
        image_writer_threads=2, encoder_threads=2)
    report = dict(source=str(source), fps=fps, lerobot_version=version('lerobot'),
                  timing='Host reception alignment, not hardware-synchronized exposure',
                  units=dict(joints='rad', gripper='calibrated aperture, 0 closed / 1 open', depth='m'), episodes=[])
    try:
        for path, metadata, grid, indices, offsets in plans:
            with h5py.File(path, 'r') as file:
                for i in range(len(grid)):
                    frame = dict(task=metadata['config']['task'])
                    for target, kind in (('observation.state','observations'),('action','actions')):
                        frame[target] = np.concatenate([file[f'streams/robot/{s}/{kind}/qpos'][indices[f'streams/robot/{s}/{kind}'][i]]
                                                       for s in metadata['sides']]).astype(np.float32)
                    for name, cfg in metadata['config']['cameras'].items():
                        for stream in cfg['streams']:
                            key = f'streams/cameras/{name}/{stream}'
                            image = file[key+'/image'][indices[key][i]]
                            if stream == 'depth':
                                scale = metadata['cameras'][name]['depth_scale_m']
                                image = (image.astype(np.float32)*scale)[...,None]
                            target = f'observation.images.{name}' if stream == 'rgb' else f'observation.depth.{name}'
                            frame[target] = image
                    dataset.add_frame(frame)
                dataset.save_episode(parallel_encoding=False)
                # Full source indices and offsets permit inspection of every pair.
                with path.open('rb') as source_file:
                    source_digest=hashlib.file_digest(source_file,'sha256').hexdigest()
                report['episodes'].append(dict(source=str(path), source_uuid=file.attrs['episode_uuid'],
                    source_sha256=source_digest,
                    source_episode_id=int(file.attrs['episode_id']), outcome=file.attrs['outcome'],
                    source_start_ns=int(file.attrs['start_ns']), source_end_ns=int(file.attrs['end_ns']),
                    grid_start_ns=int(grid[0]), frames=len(grid), metadata=metadata,
                    alignment={key:dict(indices=values.tolist(),offset_ns=offsets[key].tolist(),
                        reused_samples=int(len(values)-len(np.unique(values)))) for key,values in indices.items()}))
    finally:
        dataset.finalize()
    loaded = LeRobotDataset(repo_id, root=destination, video_backend='pyav', depth_output_unit='m')
    if len(loaded) != sum(p[2].size for p in plans):
        raise RuntimeError('LeRobot readback frame count differs')
    # Decode source boundaries of every exported episode, including image streams.
    at = 0
    for plan in plans:
        for index in (at, at+len(plan[2])-1):
            sample = loaded[index]
            if sample['action'].shape[0] != len(names):
                raise RuntimeError('LeRobot readback action schema differs')
        at += len(plan[2])
    (destination/'source_report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--repo-id', required=True)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--robot-tolerance-s', type=float, default=.04)
    parser.add_argument('--camera-tolerance-s', type=float)
    args = parser.parse_args()
    report = export(args.source,args.destination,args.repo_id,fps=args.fps,
                    robot_tolerance_s=args.robot_tolerance_s,camera_tolerance_s=args.camera_tolerance_s)
    print(json.dumps(dict(episodes=len(report['episodes']),destination=str(args.destination))))


if __name__ == '__main__':
    main()
