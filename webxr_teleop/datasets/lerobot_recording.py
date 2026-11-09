"""Native LeRobot v3 recording with disk-backed, bounded-memory capture staging.

SQLite is an attempt journal, not a second HDF5 dataset. Only accepted attempts
enter the official LeRobot writer; discard/abort never modifies training data.
"""
from io import BytesIO
import json
from pathlib import Path
import sqlite3

import numpy as np

from .alignment import alignment
from .errors import EpisodeRejected
from .export_lerobot import create_dataset, frame_at, require_lerobot, signature
from .hdf5 import json_text, ns


def _pack(value):
    buffer = BytesIO()
    np.save(buffer, value, allow_pickle=False)
    return buffer.getvalue()


class _Column:
    def __init__(self, connection, stream, column):
        self.connection, self.stream, self.column = connection, stream, column

    def __getitem__(self, index):
        if self.column == 't_ns':
            rows = self.connection.execute('SELECT t FROM samples WHERE stream=? ORDER BY t', (self.stream,))
            return np.asarray([row[0] for row in rows], dtype=np.int64)[index]
        if not isinstance(index, (int, np.integer)) or index < 0:
            raise IndexError(index)
        row = self.connection.execute('SELECT value FROM samples WHERE stream=? ORDER BY t LIMIT 1 OFFSET ?',
                                      (self.stream, int(index))).fetchone()
        if row is None:
            raise IndexError(index)
        return np.load(BytesIO(row[0]), allow_pickle=False)


class _View:
    """Minimal stream reader shared with offline HDF5 alignment/frame creation."""
    def __init__(self, episode, end_ns):
        self.episode = episode
        self.attrs = dict(start_ns=episode.start_ns, end_ns=end_ns)

    def __contains__(self, path):
        return path == 'events'

    def __getitem__(self, path):
        if path == 'metadata/json':
            return np.asarray(json_text(self.episode.metadata))
        if path.startswith('events/'):
            rows = self.episode.connection.execute('SELECT t, value FROM events ORDER BY id').fetchall()
            return np.asarray([row[0 if path.endswith('t_ns') else 1] for row in rows])
        stream, column = path.rsplit('/', 1)
        if column in ('qpos', 'image', 't_ns'):
            return _Column(self.episode.connection, stream, column)
        # alignment reads file[stream]['t_ns'].
        return {'t_ns': _Column(self.episode.connection, path, 't_ns')}


class NativeRecording:
    def __init__(self, root, config, metadata):
        require_lerobot()  # Fail before camera startup, without importing torch.
        if (Path(root) / 'accepted').exists() or list(Path(root).glob('episode_*')):
            raise ValueError('dataset_dir contains HDF5 attempts; use a new directory for LeRobot')
        self.root, self.config = Path(root), config
        self.destination = self.root / 'lerobot'
        self.attempts = self.root / 'attempts'
        self.attempts.mkdir(exist_ok=True)
        if list(self.attempts.glob('*.committing')):
            raise RuntimeError('Interrupted LeRobot commit: inspect attempts/*.committing and dataset before resuming')
        info = self.destination / 'meta/info.json'
        self.saved = json.loads(info.read_text())['total_episodes'] if info.exists() else 0
        if self.destination.exists() and not info.exists():
            raise RuntimeError('Incomplete LeRobot dataset; inspect it before resuming')
        manifest = dict(repo_id=config['lerobot']['repo_id'], fps=config['lerobot']['fps'],
                        signature=signature(dict(metadata, config=config)))
        manifest_path = self.root / 'lerobot_recording.json'
        if manifest_path.exists():
            if json.loads(manifest_path.read_text()) != manifest:
                raise ValueError('LeRobot recording schema, rig, mode, repo_id or fps changed; use a new dataset_dir')
        elif self.destination.exists():
            raise ValueError('Existing LeRobot dataset lacks recording provenance; use a new dataset_dir')
        else:
            manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')

    def next_identity(self):
        return max((int(p.name.split('.')[0].split('_')[1])
                    for p in self.attempts.glob('episode_*')), default=-1) + 1

    def episode(self, identity, start_ns, metadata):
        return NativeEpisode(self, identity, start_ns, metadata)


class NativeEpisode:
    def __init__(self, recording, identity, start_ns, metadata):
        self.recording, self.identity, self.start_ns = recording, identity, start_ns
        self.metadata, self.capture_since = metadata, start_ns
        self.last_sequences = {}
        self.path = recording.attempts / f'episode_{identity:06d}.partial.sqlite'
        if self.path.exists():
            raise FileExistsError(self.path)
        self.connection = sqlite3.connect(self.path, check_same_thread=False)
        self.connection.executescript('''
            CREATE TABLE samples (stream TEXT, t INTEGER, value BLOB, PRIMARY KEY(stream,t));
            CREATE TABLE events (id INTEGER PRIMARY KEY, t INTEGER, value TEXT);
            CREATE TABLE metadata (value TEXT);
        ''')
        self.connection.execute('INSERT INTO metadata VALUES (?)', (json_text(metadata),))
        self.closed = False
        self.event('start', start_ns)

    def _append(self, stream, timestamp, value):
        previous = self.connection.execute('SELECT MAX(t) FROM samples WHERE stream=?', (stream,)).fetchone()[0]
        if previous is not None and timestamp <= previous:
            raise ValueError(f'Nonmonotonic stream: {stream}')
        self.connection.execute('INSERT INTO samples VALUES (?,?,?)', (stream, int(timestamp), _pack(value)))

    def event(self, kind, timestamp, **details):
        self.connection.execute('INSERT INTO events(t,value) VALUES (?,?)',
                                (int(timestamp), json_text(dict(kind=kind, **details))))

    def robot(self, side, row):
        timestamp = ns(row['t'])
        if self.capture_since is None or timestamp < self.capture_since:
            return
        key = ('robot', side)
        previous = self.last_sequences.get(key)
        if previous is not None and row['seq'] != previous + 1:
            raise ValueError(f'{side}: motor sample sequence gap')
        self.last_sequences[key] = row['seq']
        qpos = np.asarray(row['q'], dtype=np.float64)
        if qpos.shape != (7,) or not np.isfinite(qpos).all():
            raise ValueError(f'{side}: invalid joint vector')
        self._append(f'streams/robot/{side}/observations', timestamp, qpos)
        action = row.get('action')
        if action and action.get('confirmed') and action['t_ns'] >= self.capture_since:
            qpos = np.asarray(action['qpos'], dtype=np.float64)
            if qpos.shape != (7,) or not np.isfinite(qpos).all():
                raise ValueError(f'{side}: invalid action vector')
            self._append(f'streams/robot/{side}/actions', action['t_ns'], qpos)

    def camera(self, name, stream, frame):
        if self.capture_since is None or frame['t_ns'] < self.capture_since:
            return
        key = ('camera', name, stream)
        previous = self.last_sequences.get(key)
        if previous is not None and frame['frame_number'] <= previous:
            raise ValueError(f'{name}/{stream}: camera frame number reset or repeated')
        if previous is not None and frame['frame_number'] != previous + 1:
            self.event('camera_gap', frame['t_ns'], camera=name, stream=stream,
                       previous_frame=previous, frame_number=frame['frame_number'])
        self.last_sequences[key] = frame['frame_number']
        cfg = self.metadata['config']['cameras'][name]
        expected = (cfg['height'], cfg['width']) + ((3,) if stream == 'rgb' else ())
        if frame['image'].shape != expected:
            raise ValueError(f'{name}: invalid image shape')
        self._append(f'streams/cameras/{name}/{stream}', frame['t_ns'], frame['image'])

    def controller(self, side, row):
        timestamp = ns(row.get('packet_received_t') or row['t'])
        if self.capture_since is not None and timestamp >= self.capture_since:
            key = ('controller', side)
            sequence = row.get('packet_sequence', timestamp)
            if self.last_sequences.get(key) != sequence:
                self.event('controller', timestamp, side=side, value=row)
                self.last_sequences[key] = sequence

    def pause(self, timestamp, reason):
        self.connection.execute('DELETE FROM samples WHERE t>=?', (int(timestamp),))
        self.capture_since = None
        self.event('pause', timestamp, reason=reason)

    def resume(self, timestamp):
        self.capture_since = timestamp
        self.last_sequences.clear()
        self.event('resume', timestamp)

    def flush(self):
        self.connection.commit()

    def close_partial(self):
        if not self.closed:
            self.connection.commit()
            self.connection.close()
            self.closed = True

    def finish(self, status, end_ns, *, outcome='unknown', reason=''):
        if status not in ('saved', 'discarded', 'aborted') or outcome not in ('unknown', 'success', 'failure'):
            raise ValueError('Invalid episode status/outcome')
        self.connection.execute('DELETE FROM samples WHERE t>=?', (int(end_ns),))
        self.event('end', end_ns, status=status, outcome=outcome, reason=reason)
        self.flush()
        if status != 'saved':
            self.close_partial()
            target = self.path.with_name(f'episode_{self.identity:06d}.{status}.sqlite')
            self.path.rename(target)
            return target
        options = self.recording.config['lerobot']
        view = _View(self, end_ns)
        try:
            grid, indices, offsets = alignment(view, options['fps'], options['robot_tolerance_s'],
                                                options['camera_tolerance_s'])
        except ValueError as exc:
            # Validation precedes the commit marker and every dataset write.
            # Keep the original samples for inspection; this is not an accepted
            # episode and must not prevent a subsequent capture attempt.
            self.event('save_rejected', end_ns, reason=str(exc))
            self.close_partial()
            target = self.path.with_name(f'episode_{self.identity:06d}.rejected.sqlite')
            self.path.rename(target)
            self.path = target
            raise EpisodeRejected(str(exc), target) from exc
        destination = self.recording.destination
        marker = self.path.with_name(f'episode_{self.identity:06d}.committing')
        marker.write_text(json.dumps(dict(destination=str(destination), previous_episodes=self.recording.saved)) + '\n')
        dataset = None
        try:
            if (destination / 'meta/info.json').exists():
                from lerobot.configs import RGBEncoderConfig
                from lerobot.datasets.lerobot_dataset import LeRobotDataset
                dataset = LeRobotDataset.resume(options['repo_id'], root=destination, video_backend='pyav',
                                                 encoder_threads=2, streaming_encoding=True, encoder_queue_maxsize=8,
                                                 rgb_encoder=RGBEncoderConfig(vcodec='h264', pix_fmt='yuv420p',
                                                                              crf=18, preset='fast'))
            else:
                dataset = create_dataset(destination, options['repo_id'], self.metadata, options['fps'],
                                         streaming_encoding=True)
            for i in range(len(grid)):
                dataset.add_frame(frame_at(view, self.metadata, indices, i))
            dataset.save_episode(parallel_encoding=False)
        finally:
            if dataset is not None:
                dataset.finalize()
        info = json.loads((destination / 'meta/info.json').read_text())
        if info['codebase_version'] != 'v3.0' or info['total_episodes'] != self.recording.saved + 1:
            raise RuntimeError('LeRobot commit verification failed')
        report = dict(episode_index=self.recording.saved, attempt=self.identity, outcome=outcome, reason=reason,
                      start_ns=self.start_ns, end_ns=end_ns, frames=len(grid), metadata=self.metadata,
                      timing='Host reception alignment, not hardware-synchronized exposure',
                      units=dict(joints='rad', gripper='calibrated aperture, 0 closed / 1 open', depth='m'),
                      events=[dict(t_ns=t, **json.loads(raw)) for t, raw in
                              self.connection.execute('SELECT t,value FROM events ORDER BY id')],
                      alignment={k:dict(indices=v.tolist(),offset_ns=offsets[k].tolist()) for k,v in indices.items()})
        report_path = self.path.with_name(f'episode_{self.identity:06d}.saved.json')
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        self.recording.saved += 1
        self.close_partial()
        self.path.unlink()
        marker.unlink()
        return destination
