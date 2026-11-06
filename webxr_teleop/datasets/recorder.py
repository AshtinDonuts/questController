"""Camera and HDF5 process. Has no motor driver imports or motion authority."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import json
import os
from pathlib import Path
import queue
import re
import signal
import socket
import time

from .config import load_config
from .cameras import Camera
from .hdf5 import Episode
from .viewer import Preview
from ..hardware.ipc import send, receive


def run(config, metadata, control, data, parent_pid):
    root = Path(config['dataset_dir'])
    root.mkdir(parents=True, exist_ok=True)
    lock = (root/'.recording.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    cameras = {}
    preview = None
    episode = None
    stopping = False
    state = dict(phase='warming', saved=len(list((root/'accepted').glob('*.hdf5'))), error=None)
    ack = None
    completed = set()
    pending_finish = None
    finalizer = ThreadPoolExecutor(max_workers=1)
    finishing = None
    last_seq = 0
    last_status = last_flush = time.monotonic()
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        cameras = {name: Camera(name, camera, config['queue_frames']*len(camera['streams']))
                   for name, camera in config['cameras'].items()}
        preview = Preview(config['display_data'], config['cameras'])
        while not stopping:
            now = time.monotonic()
            now_ns = time.monotonic_ns()
            if os.getppid() != parent_pid:
                raise RuntimeError('Supervisor disappeared')
            healthy = all(camera.healthy(now_ns) for camera in cameras.values())
            for camera in cameras.values():
                if camera.error:
                    raise RuntimeError(camera.error)
            if state['phase'] == 'warming' and healthy:
                state['phase'] = 'idle'
            if episode is not None and not healthy:
                raise RuntimeError('Required camera stream became stale')
            # Drain telemetry before processing finish; through_seq closes the
            # remaining cross-channel race without waiting in the supervisor.
            # Bound disk work per turn so a telemetry burst cannot starve
            # camera draining, control requests, or recorder heartbeats.
            for message in receive(data, limit=8):
                if message['seq'] != last_seq+1 and episode is not None:
                    raise RuntimeError('Recorder telemetry sequence gap')
                last_seq = message['seq']
                if message['kind'] == 'robot':
                    row, side = message['value'], message['side']
                    preview.put('robot', side, int(row['t']*1e9), dict(row,age_ms=1000*(now-row['t'])))
                    if episode:
                        episode.robot(side, row)
                elif message['kind'] == 'status':
                    preview.status(message['value'])
                elif message['kind'] == 'controllers' and episode:
                    for side, hand in message['hands'].items():
                        if hand:
                            episode.controller(side, hand)
                elif message['kind'] == 'event' and episode:
                    episode.event(message['event'], message['t_ns'], **message.get('details', {}))
            for name, camera in cameras.items():
                # Service each camera fairly; queue capacity is not a batch
                # size. Draining one camera completely can starve the others.
                for _ in range(len(camera.config['streams'])):
                    try:
                        stream, frame = camera.frames.get_nowait()
                    except queue.Empty:
                        break
                    display=frame['image'] if stream=='rgb' else dict(image=frame['image'],scale=camera.metadata['depth_scale_m'])
                    preview.put('image' if stream == 'rgb' else 'depth', f'{name}/{stream}', frame['t_ns'], display)
                    if episode:
                        episode.camera(name, stream, frame)
            for request in receive(control):
                identity = request['request_id']
                if identity in completed:
                    send(control, dict(state, ack=identity))
                    continue
                op = request['op']
                if op == 'start':
                    if config.get('num_episodes',0) and state['saved'] >= config['num_episodes']:
                        ack=identity
                        completed.add(identity)
                        state['rejected']='Recording complete: saved episode target reached'
                        send(control,dict(state,ack=ack))
                        continue
                    if episode is not None or pending_finish is not None or state['phase'] != 'idle' and state['phase'] != 'finished' or not healthy:
                        ack=identity
                        completed.add(identity)
                        state['rejected']='Recorder start rejected: not ready; retry when streams recover'
                        send(control,dict(state,ack=ack))
                        continue
                    numbers = [int(m.group(1)) for p in root.rglob('episode_*')
                               if (m := re.fullmatch(r'episode_(\d+)\.(?:hdf5|partial)', p.name))]
                    index = max(numbers, default=-1)+1
                    snapshot = dict(metadata, config=config, cameras={n:c.metadata for n,c in cameras.items()})
                    episode = Episode(root, index, request['start_ns'], snapshot)
                    state.pop('rejected',None)
                    state.pop('end_ns',None)
                    state.pop('pause_ns',None)
                    state.update(phase='recording', episode=index, start_ns=request['start_ns'], paused_ns=0)
                elif op == 'pause':
                    if episode is not None and state['phase']=='recording':
                        episode.pause(request['pause_ns'],request.get('reason','tracking lost'))
                        state.update(phase='paused',pause_ns=request['pause_ns'])
                elif op == 'resume':
                    if episode is not None and state['phase']=='paused' and healthy:
                        episode.resume(request['resume_ns'])
                        state['paused_ns']+=request['resume_ns']-state.pop('pause_ns')
                        state['phase']='recording'
                elif op == 'finish':
                    if finishing is not None:
                        # Capture already ended and commit owns the file.
                        # A later stop still stops motors, but cannot rewrite it.
                        completed.add(identity)
                        send(control,dict(state,ack=identity))
                        continue
                    if pending_finish and pending_finish[0]['request_id'] == identity:
                        continue
                    if pending_finish and request.get('status') == 'aborted':
                        pending_finish = None  # A stop can supersede a pending save.
                    pending_finish = (request, now)
                    state['end_ns']=request['end_ns']
                    state['phase'] = 'saving'
                    continue
                elif op == 'shutdown':
                    stopping = True
                    break
                else:
                    raise ValueError(f'Unknown recorder operation: {op}')
                ack = identity
                completed.add(identity)
            if pending_finish:
                request, requested_at = pending_finish
                # A retransmission must not extend this deadline.
                if finishing is None and (last_seq >= request['through_seq'] or now-requested_at > .5):
                    status = request['status']
                    if last_seq < request['through_seq']:
                        status = 'aborted'
                    if episode:
                        # Transfer sole HDF5 ownership; keep draining cameras
                        # and publishing heartbeats during disk finalization.
                        finishing=(finalizer.submit(episode.finish,status,request['end_ns'],
                            outcome=request.get('outcome','unknown'),reason=request.get('reason','')),request,status)
                        episode = None
                    else:
                        state.update(phase='finished',last_status=status)
                        ack=request['request_id'];completed.add(ack);pending_finish=None
            if finishing is not None and finishing[0].done():
                future,request,status=finishing
                path=future.result()
                state['saved'] += status=='saved'
                state['path']=str(path)
                state.update(phase='finished', last_status=status)
                ack = request['request_id']
                completed.add(ack)
                pending_finish = None
                finishing = None
            if episode and now-last_flush >= config['flush_interval_s']:
                episode.file.flush()
                last_flush = now
            state['cameras'] = {n: dict(healthy=c.healthy(now_ns), error=c.error) for n,c in cameras.items()}
            if now-last_status >= .05:
                send(control, dict(state, ack=ack))
                last_status = now
            time.sleep(.002)
    except Exception as exc:
        end_ns=time.monotonic_ns()
        state.update(phase='failed', error=f'{type(exc).__name__}: {exc}', end_ns=end_ns)
        # Publish the stopped boundary before potentially slow disk cleanup.
        send(control, dict(state, ack=ack))
        if episode:
            try:
                episode.finish('aborted', end_ns, reason=state['error'])
                episode = None
            except Exception:
                episode.close_partial()
        send(control, dict(state, ack=ack))
        raise
    finally:
        finalizer.shutdown(wait=True)
        if episode:
            try:
                episode.finish('aborted', time.monotonic_ns(), reason='Recorder shutdown')
            except Exception:
                episode.close_partial()
        for camera in cameras.values():
            camera.close()
        if preview:
            preview.close()
        lock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--control-fd', type=int, required=True)
    parser.add_argument('--data-fd', type=int, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    args = parser.parse_args()
    metadata = json.loads(args.metadata.read_text())
    config = load_config(args.config, mock=metadata['mode'] == 'mock')
    control, data = socket.socket(fileno=args.control_fd), socket.socket(fileno=args.data_fd)
    try:
        run(config, metadata, control, data, args.parent_pid)
    finally:
        control.close()
        data.close()


if __name__ == '__main__':
    main()
