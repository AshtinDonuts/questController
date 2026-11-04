"""Offline native HDF5 replay in Rerun. Never opens robot or camera devices."""
import argparse
import heapq
import json
import os
from pathlib import Path
import socket
import sys

import h5py

from .hdf5 import SCHEMA
from .viewer import blueprint
from ..hardware.core import JOINTS


def episodes(source):
    source = Path(source)
    if source.is_file():
        return [source]
    directory = source / 'accepted' if (source / 'accepted').is_dir() else source
    return sorted(directory.glob('*.hdf5'))


def summary(path):
    with h5py.File(path, 'r') as file:
        if file.attrs.get('schema') != SCHEMA or Path(path).suffix != '.hdf5':
            raise ValueError(f'Expected a completed {SCHEMA} file: {path}')
        return dict(path=str(path), status=str(file.attrs['status']),
                    outcome=str(file.attrs.get('outcome', 'unknown')),
                    duration_s=(int(file.attrs['end_ns']) - int(file.attrs['start_ns'])) / 1e9)


def log_episode(path, rr):
    """Stream images one at a time, preserving each stream's native timestamps."""
    info = summary(path)
    with h5py.File(path, 'r') as file:
        metadata = json.loads(file['metadata/json'][()])
        start = int(file.attrs['start_ns'])
        rr.send_blueprint(blueprint(metadata['config']['cameras']))
        rr.log('recording/status', rr.TextDocument(
            f"# {Path(path).name}\n\n{metadata['config'].get('task', '')}\n\n"
            f"{info['status']} / {info['outcome']} · {info['duration_s']:.2f} s\n\n"
            'Offline replay. Joint angles: radians; gripper: 0 closed, 1 open.',
            media_type='text/markdown'), timeless=True)
        streams = []
        for side, arm in file.get('streams/robot', {}).items():
            for stream, label in (('observations', 'measured'), ('actions', 'command')):
                if stream in arm:
                    streams.append(('robot', side, label, arm[stream]))
        for name, camera in file.get('streams/cameras', {}).items():
            for stream, group in camera.items():
                streams.append(('camera', name, stream, group))
        if 'events' in file:
            streams.append(('events', '', '', file['events']))

        # Merge native timelines without loading image arrays into memory.
        # Sending complete streams one after another makes live ingestion appear
        # sequential and repeatedly moves the episode playhead backwards.
        pending = []
        for stream_id, (_, _, _, group) in enumerate(streams):
            if len(group['t_ns']):
                heapq.heappush(pending, (int(group['t_ns'][0]), stream_id, 0))
        while pending:
            timestamp, stream_id, index = heapq.heappop(pending)
            kind, name, stream, group = streams[stream_id]
            rr.set_time_nanos('episode', timestamp - start)
            if kind == 'robot':
                for joint, value in zip(JOINTS + ('gripper',), group['qpos'][index]):
                    rr.log(f'robot/{name}/{joint}/{stream}', rr.Scalar(float(value)))
            elif kind == 'camera':
                frame = group['image'][index]
                if stream == 'depth':
                    scale = metadata['cameras'][name]['depth_scale_m']
                    value = rr.DepthImage(frame, meter=1 / scale)
                else:
                    value = rr.Image(frame)
                rr.log(f'cameras/{name}/{stream}', value)
            else:
                event = group['json'][index]
                rr.log('events', rr.TextLog(event.decode() if isinstance(event, bytes) else event))
            index += 1
            if index < len(group['t_ns']):
                heapq.heappush(pending, (int(group['t_ns'][index]), stream_id, index))
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', nargs='?', type=Path,
                        help='Episode, accepted directory, or dataset directory (default: recording.json dataset)')
    parser.add_argument('--list', action='store_true', help='List episodes without opening a viewer')
    parser.add_argument('--save', type=Path, metavar='FILE.rrd', help='Export a Rerun recording instead of opening a window')
    args = parser.parse_args()
    source = args.source
    if source is None:
        config = Path(__file__).resolve().parents[2] / 'recording.json'
        source = config.parent / json.loads(config.read_text())['dataset_dir']
    paths = episodes(source)
    if not paths:
        parser.error(f'No completed HDF5 episodes found in {source}')
    try:
        if args.list:
            for path in paths:
                print(json.dumps(summary(path)))
            return
        path = max(paths, key=lambda item: item.stat().st_mtime_ns)
        summary(path)  # Reject unsupported files before starting the viewer.
        import rerun as rr
        os.environ['PATH'] = str(Path(sys.executable).parent) + os.pathsep + os.environ.get('PATH', '')
        rr.init('Quest episode replay')
        if args.save:
            if args.save.exists():
                parser.error(f'Output already exists: {args.save}')
            rr.save(str(args.save))
        else:
            with socket.socket() as listener:
                listener.bind(('127.0.0.1', 0))
                port = listener.getsockname()[1]
            rr.spawn(port=port, memory_limit='2GB', hide_welcome_screen=True)
        print(f'Loading {path}', flush=True)
        log_episode(path, rr)
        rr.disconnect()
        print(f'Saved {args.save}' if args.save else 'Loaded. Select the episode timeline; use Play or drag the playhead.')
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
