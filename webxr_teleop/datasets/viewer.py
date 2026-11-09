"""Disposable Rerun display. Preview loss never drops source recording data."""
import inspect
import multiprocessing as mp
import os
from pathlib import Path
import queue
import socket
import sys

from ..hardware.core import JOINTS
from ..hardware.recording_status import status_markdown


def blueprint(cameras):
    import rerun.blueprint as rrb

    rows = [rrb.TextDocumentView(name='Recording progress',origin='/recording/status')]
    for name, camera in cameras.items():
        rows.append(rrb.Horizontal(*(
            rrb.Spatial2DView(
                name=f'{name} {stream.upper()}',
                origin=f'/cameras/{name}/{stream}',
                contents=[f'/cameras/{name}/{stream}'],
            )
            for stream in camera['streams']
        )))
    rows.append(rrb.TimeSeriesView(name='Robot feedback', origin='/robot'))
    # Rerun 0.22 (NumPy 2) renamed the 0.16 acquisition API's argument.
    auto_views = 'auto_views' if 'auto_views' in inspect.signature(rrb.Blueprint).parameters else 'auto_space_views'
    return rrb.Blueprint(rrb.Vertical(*rows), auto_layout=False, **{auto_views: False})


def run(messages, cameras, statuses):
    # Absolute-interpreter launches do not activate the conda environment's
    # bin directory; Rerun locates its matching viewer through PATH.
    os.environ['PATH']=str(Path(sys.executable).parent)+os.pathsep+os.environ.get('PATH','')
    import rerun as rr
    rr.init('Quest dataset recording')
    with socket.socket() as listener:
        listener.bind(('127.0.0.1',0))
        port=listener.getsockname()[1]
    rr.spawn(port=port,memory_limit='512MB',hide_welcome_screen=True)
    rr.send_blueprint(blueprint(cameras))
    rr.log('recording/status',rr.TextDocument('# CONNECTING\n\nWaiting for recording status.',media_type='text/markdown'))
    while True:
        try:
            status=statuses.get_nowait()
        except queue.Empty:
            pass
        else:
            rr.log('recording/status',rr.TextDocument(status_markdown(status),media_type='text/markdown'))
        try:
            item = messages.get(timeout=.05)
        except queue.Empty:
            continue
        if item is None:
            return
        kind, name, timestamp, value = item
        rr.set_time_seconds('session', timestamp/1e9)
        if kind == 'image':
            rr.log(f'cameras/{name}', rr.Image(value))
        elif kind == 'depth':
            rr.log(f'cameras/{name}', rr.DepthImage(value['image'],meter=1/value['scale']))
        elif kind == 'robot':
            goal=(value.get('action') or {}).get('qpos',value['goal'])
            for joint, measured, target in zip(JOINTS+('gripper',),value['q'],goal):
                rr.log(f'robot/{name}/{joint}/measured', rr.Scalar(measured))
                rr.log(f'robot/{name}/{joint}/command', rr.Scalar(target))
            rr.log(f'robot/{name}/feedback_age_ms', rr.Scalar(value.get('age_ms', 0)))


class Preview:
    def __init__(self, enabled, cameras):
        self.process = None
        if enabled:
            ctx = mp.get_context('spawn')
            self.messages = ctx.Queue(4)
            self.statuses = ctx.Queue(1)
            self.process = ctx.Process(target=run, args=(self.messages, cameras, self.statuses), daemon=True)
            self.process.start()

    def put(self, *item):
        if self.process is not None and self.process.is_alive():
            try:
                self.messages.put_nowait(item)
            except queue.Full:
                pass

    def status(self, value):
        # Keep progress independent of the busy image/telemetry preview queue.
        if self.process is not None and self.process.is_alive():
            try:
                self.statuses.get_nowait()
            except queue.Empty:
                pass
            try:
                self.statuses.put_nowait(value)
            except queue.Full:
                pass

    def close(self):
        if self.process is not None:
            try:
                self.messages.put_nowait(None)
            except queue.Full:
                pass
            self.process.join(timeout=1)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1)
            self.messages.cancel_join_thread()
            self.messages.close()
            self.statuses.cancel_join_thread()
            self.statuses.close()
