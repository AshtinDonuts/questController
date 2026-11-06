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
        self.sample_seq += 1
        if not send(self.data, dict(seq=self.sample_seq, kind=kind, **value)):
            self.error = 'Recorder transport overflow'

    def poll(self):
        now = time.monotonic()
        try:
            messages = receive(self.control)
        except (OSError, ConnectionError, ValueError) as exc:
            messages = []
            self.error = self.error or str(exc)
        for message in messages:
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
            if now-self.pending_since > max(5, self.timeout):
                self.error = self.error or 'Recorder transition acknowledgement timed out'
        if self.process.poll() is not None and not self.error:
            self.error = 'Recorder process exited'
        if self.process.poll() is not None:
            self.phase='failed'
        elif now-self.last_status > self.timeout:
            self.error = self.error or 'Recorder heartbeat expired'
        if self.error and self.end_ns is None and self.start_ns is not None:
            self.end_ns = time.monotonic_ns()
            self.status = dict(self.status, end_ns=self.end_ns)
        return self.status

    def close(self):
        # This wait runs only after motor workers have entered shutdown holds.
        send(self.control, dict(op='shutdown', request_id=uuid.uuid4().hex, through_seq=self.sample_seq))
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.control.close()
        self.data.close()
