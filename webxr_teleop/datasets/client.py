"""Dependency-free nonblocking supervisor interface to the recording process."""
import os
from pathlib import Path
import subprocess
import time
import uuid

from ..hardware.ipc import pair, send, receive


class RecorderClient:
    def __init__(self, config_path, metadata_path, python, timeout):
        self.control, child_control = pair()
        self.data, child_data = pair()
        env = os.environ.copy()
        for name in ('PYTHONPATH', 'PYTHONHOME', 'PYTHONUSERBASE'):
            env.pop(name, None)
        env['PYTHONNOUSERSITE'] = '1'
        self.process = subprocess.Popen([python, '-m', 'webxr_teleop.datasets.recorder',
            '--config', str(config_path), '--metadata', str(metadata_path),
            '--control-fd', str(child_control.fileno()), '--data-fd', str(child_data.fileno()),
            '--parent-pid', str(os.getpid())],
            pass_fds=(child_control.fileno(), child_data.fileno()), env=env,
            cwd=Path(__file__).resolve().parents[2], start_new_session=True)
        child_control.close()
        child_data.close()
        self.timeout = timeout
        self.last_status = time.monotonic()
        # Camera discovery can block after its first "checking connection"
        # status. Bound the entire startup, then use the runtime heartbeat
        # deadline only after the recorder has completed camera warmup.
        self.startup_started = self.last_status
        self.startup_complete = False
        self.awaiting_first_status = True
        self.pending = None
        self.phase = 'warming'
        self.status = dict(phase='warming', saved=0)
        self.error = None
        self.sample_seq = 0
        self.start_ns = None
        self.end_ns = None

    def request(self, op, **kwargs):
        if self.pending is not None:
            raise ValueError('Recording transition still pending')
        if op == 'finish' and self.end_ns is not None:
            kwargs['end_ns'] = min(self.end_ns, kwargs['end_ns'])
        self.pending = dict(op=op, request_id=uuid.uuid4().hex, through_seq=self.sample_seq, **kwargs)
        self.pending_since = time.monotonic()
        if op == 'start':
            self.end_ns = None
            self.phase = 'starting'
        elif op == 'pause':
            self.phase = 'pausing'
            self.status = dict(self.status, pause_ns=kwargs['pause_ns'])
        elif op == 'resume':
            self.phase = 'resuming'
        elif op == 'finish':
            self.end_ns = kwargs['end_ns']
            self.status = dict(self.status, end_ns=self.end_ns)
            self.phase = 'saving'
        send(self.control, self.pending)

    def pause(self, reason):
        if self.phase not in ('starting', 'recording', 'resuming'):
            return
        # Ordered control datagrams retain the start before this pause.
        self.pending = None
        self.request('pause', pause_ns=time.monotonic_ns(), reason=reason)

    def abort(self, reason):
        if self.phase not in ('starting', 'recording', 'saving', 'pausing', 'paused', 'resuming'):
            return
        if self.pending and self.pending.get('op')=='finish' and self.pending.get('status')=='aborted':
            return
        # Abort supersedes an unacknowledged start/save and always wins in recorder.
        self.pending = None
        self.request('finish', status='aborted', end_ns=time.monotonic_ns(), reason=reason)

    def sample(self, kind, **value):
        if self.error:
            return
        # Only successfully queued messages belong to the ordered stream and
        # finish's through_seq barrier. Outside capture these are preview-only:
        # encoder finalization may briefly saturate the socket without losing
        # any episode sample. Keep capture/transition delivery strict.
        seq = self.sample_seq + 1
        if send(self.data, dict(seq=seq, kind=kind, **value)):
            self.sample_seq = seq
        elif self.phase not in ('warming', 'idle', 'saving', 'finished'):
            self.error = 'Recorder transport overflow'

    def poll(self):
        now = time.monotonic()
        try:
            messages = receive(self.control)
        except (OSError, ConnectionError, ValueError) as exc:
            messages = []
            self.error = self.error or str(exc)
        for message in messages:
            self.awaiting_first_status = False
            if message['phase'] != 'warming':
                self.startup_complete = True
            self.last_status = now
            previous_pause=self.status.get('pause_ns')
            self.status = message
            if self.pending and self.pending['op']=='pause' and previous_pause is not None:
                self.status=dict(message,pause_ns=previous_pause)
            if self.end_ns is not None:
                self.status = dict(message, end_ns=self.end_ns)
            elif message.get('end_ns') is not None:
                self.end_ns = message['end_ns']
            self.start_ns = message.get('start_ns')
            if message['phase']=='failed':
                self.pending=None
            if self.pending and message.get('ack') == self.pending['request_id']:
                self.pending = None
            if self.pending is None:
                self.phase = message['phase']
            if message.get('error'):
                self.error = self.error or message['error']
        if self.pending:
            send(self.control, self.pending)
            saving = self.pending['op'] == 'finish' and self.status.get('phase') == 'saving'
            if not saving and now-self.pending_since > max(5, self.timeout):
                self.error = self.error or 'Recorder transition acknowledgement timed out'
        if self.process.poll() is not None and not self.error:
            self.error = 'Recorder process exited'
        if self.process.poll() is not None:
            self.phase='failed'
        elif not self.startup_complete:
            # Heartbeats must not indefinitely extend an unready startup.
            if now-self.startup_started > max(10, self.timeout):
                self.error = self.error or self.diagnostic_error(
                    'Recorder startup heartbeat timed out' if self.awaiting_first_status
                    else 'Recorder camera startup timed out')
        elif now-self.last_status > self.timeout:
            self.error = self.error or self.diagnostic_error('Recorder heartbeat expired')
        if self.error and self.end_ns is None and self.start_ns is not None:
            self.end_ns = time.monotonic_ns()
            self.status = dict(self.status, end_ns=self.end_ns)
        return self.status

    def diagnostic_error(self, reason):
        cameras = self.status.get('cameras', {})
        details = [f"{name} ({c.get('backend', 'camera')} device {c.get('device', 'unknown')}): "
                   f"{c.get('error') or c.get('stage', 'no fresh frames')}"
                   for name, c in cameras.items() if not c.get('healthy')]
        return reason + ('; cameras not ready: ' + '; '.join(details)
                         + '. Check camera USB connections/power and configured device identifiers.' if details else '')

    def close(self):
        # This wait runs only after motor workers have entered shutdown holds.
        # Video finalization may outlast ordinary recorder shutdown.
        send(self.control, dict(op='shutdown', request_id=uuid.uuid4().hex, through_seq=self.sample_seq))
        try:
            self.process.wait(timeout=120 if self.phase == 'saving' else 5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.control.close()
        self.data.close()
