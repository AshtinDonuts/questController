"""Recording configuration, deliberately independent of acquisition libraries."""
import json
import math
from pathlib import Path
import re


def number(value, name, minimum=0, integer=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= minimum or (integer and int(value) != value)):
        raise ValueError(f'{name} must be a finite number greater than {minimum}')
    return int(value) if integer else float(value)


def load_config(path, *, mock=False):
    path = Path(path).resolve()
    cfg = json.loads(path.read_text())
    allowed = {'dataset_dir', 'task', 'cameras', 'display_data', 'episode_time_s',
               'num_episodes', 'heartbeat_timeout_s', 'queue_frames', 'flush_interval_s'}
    if not isinstance(cfg, dict) or set(cfg)-allowed:
        raise ValueError('Unknown recording configuration fields')
    for key in ('dataset_dir', 'task'):
        if not isinstance(cfg.get(key), str) or not cfg[key].strip():
            raise ValueError(f'{key} must be a nonempty string')
    if not isinstance(cfg.get('cameras'), dict):
        raise ValueError('cameras must be explicit: a named camera map, or {} for proprioception only')
    cfg['dataset_dir'] = str((path.parent / cfg['dataset_dir']).resolve())
    cfg.setdefault('display_data', True)
    if not isinstance(cfg['display_data'], bool):
        raise ValueError('display_data must be boolean')
    cfg['heartbeat_timeout_s'] = number(cfg.get('heartbeat_timeout_s', 1), 'heartbeat_timeout_s', .2)
    cfg['queue_frames'] = number(cfg.get('queue_frames', 8), 'queue_frames', integer=True)
    cfg['flush_interval_s'] = number(cfg.get('flush_interval_s', 1), 'flush_interval_s')
    limit = cfg.get('episode_time_s', 0)
    if isinstance(limit, bool) or not isinstance(limit, (int, float)) or not math.isfinite(limit) or limit < 0:
        raise ValueError('episode_time_s must be finite and nonnegative')
    cfg['episode_time_s'] = float(limit)
    count = cfg.get('num_episodes', 0)
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise ValueError('num_episodes must be a nonnegative integer')
    cfg['num_episodes'] = count
    devices = set()
    for name, camera in cfg['cameras'].items():
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name) or not isinstance(camera, dict):
            raise ValueError('Camera names must be simple identifiers')
        if set(camera)-{'backend', 'device', 'width', 'height', 'fps', 'streams', 'max_gap_s'}:
            raise ValueError(f'{name}: unknown camera setting')
        backend = camera.get('backend')
        if backend not in ('realsense', 'opencv', 'synthetic') or (backend == 'synthetic' and not mock):
            raise ValueError(f'{name}: unsupported backend (synthetic requires mock motors)')
        if backend != 'synthetic':
            device = camera.get('device')
            if isinstance(device, bool) or not isinstance(device, (str, int)) or str(device) == '':
                raise ValueError(f'{name}: explicit device identifier required')
            camera['device'] = str(device) if backend == 'realsense' else device
            identity = (backend, str(camera['device']))
            if identity in devices:
                raise ValueError('Configure all streams of a physical camera in one entry')
            devices.add(identity)
        for key in ('width', 'height', 'fps'):
            camera[key] = number(camera.get(key), f'{name}.{key}', integer=True)
        streams = camera.get('streams')
        if (not isinstance(streams, list) or not streams or len(set(streams)) != len(streams)
                or any(s not in ('rgb', 'depth') for s in streams)
                or (backend == 'opencv' and streams != ['rgb'])):
            raise ValueError(f'{name}: specify supported streams explicitly')
        camera['max_gap_s'] = number(camera.get('max_gap_s', max(.2, 3/camera['fps'])), f'{name}.max_gap_s')
    return cfg
