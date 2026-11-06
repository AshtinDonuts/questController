"""Single-writer, lossless native-stream episode files. No motor access."""
from datetime import datetime, timezone, timedelta
import json
import os
from pathlib import Path
import time
import uuid

import h5py
import numpy as np

SCHEMA = 'quest_hdf5.v1'


def json_text(value):
    return json.dumps(value, allow_nan=False, separators=(',', ':'))


def ns(seconds):
    return int(round(seconds * 1_000_000_000))


class Episode:
    def __init__(self, root, identity, start_ns, metadata):
        self.root = Path(root)
        self.identity = identity
        self.start_ns = start_ns
        self.capture_since = start_ns
        self.pause_ns = None
        self.paused_ns = 0
        self.path = self.root / f'episode_{identity:06d}.partial'
        self.file = h5py.File(self.path, 'x')
        clock_ns=time.monotonic_ns()
        clock_utc=datetime.now(timezone.utc)
        self.file.attrs.update(schema=SCHEMA, episode_id=identity, episode_uuid=str(uuid.uuid4()),
                               status='partial', outcome='unknown', start_ns=start_ns,
                               clock_anchor_ns=clock_ns,clock_anchor_utc=clock_utc.isoformat(),
                               start_utc=(clock_utc-timedelta(microseconds=(clock_ns-start_ns)/1000)).isoformat())
        self.file.create_dataset('metadata/json', data=json_text(metadata), dtype=h5py.string_dtype())
        self.last_sequences = {}
        self.event('start', start_ns)

    def append(self, path, **values):
        group = self.file.require_group(path)
        length = next(iter(group.values())).shape[0] if len(group) else 0
        for name, value in values.items():
            if isinstance(value, str):
                value = np.asarray(value, dtype=h5py.string_dtype())
            else:
                value = np.asarray(value)
            if name not in group:
                group.create_dataset(name, shape=(0,)+value.shape, maxshape=(None,)+value.shape,
                                     chunks=(1,)+value.shape if value.ndim else (128,),
                                     dtype=value.dtype, compression='lzf' if value.dtype.kind != 'O' else None)
            dataset = group[name]
            if dataset.shape[1:] != value.shape:
                raise ValueError(f'Stream shape changed: {path}/{name}')
            dataset.resize(length+1, axis=0)
            dataset[length] = value

    def event(self, kind, timestamp, **details):
        self.append('events', t_ns=np.int64(timestamp), json=json_text(dict(kind=kind, **details)))

    def robot(self, side, row):
        timestamp = ns(row['t'])
        if (self.capture_since is None or timestamp < self.capture_since):
            return
        key = ('robot', side)
        previous = self.last_sequences.get(key)
        if previous is not None and row['seq'] != previous+1:
            raise ValueError(f'{side}: motor sample sequence gap {previous} -> {row["seq"]}')
        self.last_sequences[key] = row['seq']
        self.append(f'streams/robot/{side}/observations', t_ns=np.int64(timestamp),
                    qpos=np.asarray(row['q'], dtype=np.float64), seq=np.int64(row['seq']),
                    epoch=np.int64(row['epoch']), json=json_text(row))
        action = row.get('action')
        if action and action.get('confirmed') and action['t_ns'] >= self.capture_since:
            self.append(f'streams/robot/{side}/actions', t_ns=np.int64(action['t_ns']),
                        qpos=np.asarray(action['qpos'], dtype=np.float64),
                        seq=np.int64(row['seq']), json=json_text(action))

    def controller(self, side, row):
        timestamp = ns(row.get('packet_received_t') or row['t'])
        if (self.capture_since is None or timestamp < self.capture_since):
            return
        key = ('controller', side)
        sequence = row.get('packet_sequence')
        sequence = timestamp if sequence is None else sequence
        if self.last_sequences.get(key) == sequence:
            return
        self.last_sequences[key] = sequence
        self.append(f'streams/controllers/{side}', t_ns=np.int64(timestamp),
                    freshness_issued_ns=np.int64(ns(row['t'])), json=json_text(row))

    def camera(self, name, stream, frame):
        if self.capture_since is None or frame['t_ns'] < self.capture_since:
            return
        key=('camera',name,stream)
        previous=self.last_sequences.get(key)
        if previous is not None and frame['frame_number']<=previous:
            raise ValueError(f'{name}/{stream}: camera frame number reset or repeated')
        if previous is not None and frame['frame_number']!=previous+1:
            self.event('camera_gap',frame['t_ns'],camera=name,stream=stream,
                       previous_frame=previous,frame_number=frame['frame_number'])
        self.last_sequences[key]=frame['frame_number']
        self.append(f'streams/cameras/{name}/{stream}', t_ns=np.int64(frame['t_ns']),
                    device_ms=np.float64(frame['device_ms']), frame_number=np.int64(frame['frame_number']),
                    image=frame['image'])

    def pause(self, timestamp, reason):
        # Remove the small cross-channel tail already written before pause arrived.
        def trim(_, obj):
            if isinstance(obj, h5py.Group) and 't_ns' in obj:
                count=int(np.searchsorted(obj['t_ns'][:], timestamp, side='left'))
                for dataset in obj.values():
                    dataset.resize(count, axis=0)
        if 'streams' in self.file:
            self.file['streams'].visititems(trim)
        self.capture_since=None
        self.pause_ns=timestamp
        self.event('pause', timestamp, reason=reason)

    def resume(self, timestamp):
        self.paused_ns+=timestamp-self.pause_ns
        self.pause_ns=None
        self.capture_since=timestamp
        self.last_sequences.clear()
        self.event('resume', timestamp)

    def finish(self, status, end_ns, *, outcome='unknown', reason=''):
        if status not in ('saved', 'discarded', 'aborted') or outcome not in ('unknown', 'success', 'failure'):
            raise ValueError('Invalid episode status/outcome')
        # Separate IPC channels can deliver a small tail before the end command.
        # Keep the exact operator boundary, never the subsequent hold trajectory.
        def trim(_, obj):
            if isinstance(obj, h5py.Group) and 't_ns' in obj:
                count = int(np.searchsorted(obj['t_ns'][:], end_ns, side='left'))
                for dataset in obj.values():
                    dataset.resize(count, axis=0)
        self.file['streams'].visititems(trim) if 'streams' in self.file else None
        self.event('end', end_ns, status=status, reason=reason)
        paused_ns=self.paused_ns+(max(0,end_ns-self.pause_ns) if self.pause_ns is not None else 0)
        self.file.attrs.update(status=status, outcome=outcome, end_ns=end_ns, reason=reason,
                               paused_ns=paused_ns, active_duration_ns=max(0,end_ns-self.start_ns-paused_ns))
        self.file.flush()
        self.file.close()
        if status == 'saved':
            try:
                validate(self.path, allow_partial_name=True)
            except Exception:
                # A failed validation never publishes an accepted file.
                raise
        target = self.root / ('accepted' if status == 'saved' else status)
        target.mkdir(exist_ok=True)
        destination = target / f'episode_{self.identity:06d}.hdf5'
        if destination.exists():
            raise FileExistsError(destination)
        with self.path.open('rb') as handle:
            os.fsync(handle.fileno())
        self.path.rename(destination)
        fd = os.open(target, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
        return destination

    def close_partial(self):
        if self.file.id.valid:
            self.file.close()


def validate(path, *, allow_partial_name=False):
    path = Path(path)
    if path.suffix != '.hdf5' and not allow_partial_name:
        raise ValueError(f'Unfinished episode: {path}')
    with h5py.File(path, 'r') as file:
        if file.attrs.get('schema') != SCHEMA or file.attrs.get('status') != 'saved':
            raise ValueError('Expected a saved quest_hdf5.v1 episode')
        metadata = json.loads(file['metadata/json'][()])
        start, end = int(file.attrs['start_ns']), int(file.attrs['end_ns'])
        if end <= start:
            raise ValueError('Empty episode interval')
        for side in metadata['sides']:
            for stream in ('observations', 'actions'):
                group = file[f'streams/robot/{side}/{stream}']
                if group['qpos'].shape[1:] != (7,) or not np.isfinite(group['qpos'][:]).all():
                    raise ValueError(f'{side}: invalid joint vector')
        for name, camera in metadata['config']['cameras'].items():
            for stream in camera['streams']:
                group = file[f'streams/cameras/{name}/{stream}']
                shape = (camera['height'], camera['width']) + ((3,) if stream == 'rgb' else ())
                if group['image'].shape[1:] != shape:
                    raise ValueError(f'{name}: invalid image shape')
        counts = {}
        def check(name, obj):
            if isinstance(obj, h5py.Group) and 't_ns' in obj:
                times = obj['t_ns'][:]
                if (not len(times) or np.any(np.diff(times) <= 0) or times[0] < start or times[-1] >= end
                        or any(len(d) != len(times) for d in obj.values())):
                    raise ValueError(f'Invalid stream lengths/timestamps: {name}')
                counts[name] = len(times)
        file['streams'].visititems(check)
        return dict(path=str(path), episode_id=int(file.attrs['episode_id']), counts=counts)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Validate accepted native HDF5 demonstrations')
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    paths = [args.source] if args.source.is_file() else sorted((args.source/'accepted').glob('*.hdf5'))
    if not paths:
        parser.error('No accepted episodes found')
    for path in paths:
        print(json.dumps(validate(path)))


if __name__ == '__main__':
    main()
