"""Offline causal alignment shared by training bridges; timestamps stay int64."""
import json

import numpy as np


def aligned_segments(file, fps=30, robot_tolerance_s=.04, camera_tolerance_s=None, *,
                     sides=None, cameras=None, include_depth=True):
    """Causal observations, followed by each arm's next confirmed cycle target."""
    if isinstance(fps, bool) or not isinstance(fps, int) or fps <= 0:
        raise ValueError('fps must be a positive integer')
    if not np.isfinite(robot_tolerance_s) or robot_tolerance_s <= 0:
        raise ValueError('robot tolerance must be finite and positive')
    metadata = json.loads(file['metadata/json'][()])
    streams = {}
    for side in (metadata['sides'] if sides is None else sides):
        for name in ('observations', 'actions'):
            key = f'streams/robot/{side}/{name}'
            streams[key] = (file[key]['t_ns'][:], name == 'actions', robot_tolerance_s)
    for name in (metadata['config']['cameras'] if cameras is None else cameras):
        cfg = metadata['config']['cameras'][name]
        for stream in cfg['streams']:
            if stream == 'depth' and not include_depth:
                continue
            key = f'streams/cameras/{name}/{stream}'
            tolerance = 2/cfg['fps'] if camera_tolerance_s is None else camera_tolerance_s
            if not np.isfinite(tolerance) or tolerance <= 0:
                raise ValueError('camera tolerance must be finite and positive')
            streams[key] = (file[key]['t_ns'][:], False, tolerance)
    # A paused episode retains native timestamps, with explicit capture gaps.
    # Build each segment independently so alignment never crosses a pause.
    intervals=[]
    segment_start=int(file.attrs['start_ns'])
    if 'events' in file:
        for timestamp,raw in zip(file['events/t_ns'][:],file['events/json'][:]):
            kind=json.loads(raw)['kind']
            if kind=='pause' and segment_start is not None:
                intervals.append((segment_start,int(timestamp)))
                segment_start=None
            elif kind=='resume':
                segment_start=int(timestamp)
    if segment_start is not None:
        intervals.append((segment_start,int(file.attrs['end_ns'])))
    grids=[]
    for lower,upper in intervals:
        coverage=[t[(t>=lower)&(t<upper)] for t,_,_ in streams.values()]
        if any(not len(t) for t in coverage):
            continue
        begin=max(int(t[0]) for t in coverage)
        end=min(int(t[-1]) for t in coverage)
        if end<=begin:
            continue
        count=int((end-begin)*fps//1_000_000_000)+1
        part=begin+np.rint(np.arange(count)*1e9/fps).astype(np.int64)
        grids.append(part[part<=end])
    if not grids:
        raise ValueError('No common valid stream coverage')
    grid=np.concatenate(grids)
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
    segments = []
    at = 0
    for part in grids:
        stop = at + len(part)
        segments.append((part, {k: v[at:stop] for k, v in indices.items()},
                         {k: v[at:stop] for k, v in offsets.items()}))
        at = stop
    return segments


def alignment(file, fps=30, robot_tolerance_s=.04, camera_tolerance_s=None):
    """Align the full recording; retained for the general LeRobot v3 exporter."""
    segments = aligned_segments(file, fps, robot_tolerance_s, camera_tolerance_s)
    return (np.concatenate([s[0] for s in segments]),
            {k: np.concatenate([s[1][k] for s in segments]) for k in segments[0][1]},
            {k: np.concatenate([s[2][k] for s in segments]) for k in segments[0][2]})
