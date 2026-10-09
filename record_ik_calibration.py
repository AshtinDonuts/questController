#!/usr/bin/env python3
"""Read-only D405 + X11 screen + ROS recording. See CALIBRATION.md."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import importlib
import json
from pathlib import Path
import queue
import shutil
import signal
import threading
import time


def protocol(side):
    stages = [(5, 'baseline', 'Release grip; hold controller still at neutral.')]
    for axis in ('up', 'forward', 'right'):
        for repeat in range(1, 4):
            prefix = f'{axis}_{repeat}'
            stages += [(4, prefix + '_reset', 'Release grip; return to neutral.'),
                       (3, prefix + '_zero', 'Hold grip; keep still.'),
                       (3, prefix + '_out', f'Keep grip held; move {axis} 5 cm slowly; keep orientation fixed.'),
                       (3, prefix + '_hold', 'Hold still, grip held.'),
                       (3, prefix + '_back', 'Keep grip held; return slowly to neutral.'),
                       (2, prefix + '_end', 'Hold still, grip held.')]
    for axis in ('roll', 'pitch', 'yaw'):
        stages += [(4, axis + '_reset', 'Release grip; return to neutral.'),
                   (3, axis + '_zero', 'Hold grip; keep still.'),
                   (6, axis + '_positive', f'Keep grip held; rotate +15 degrees in {axis}, then hold; minimize translation.'),
                   (6, axis + '_negative', f'Keep grip held; rotate through neutral to -15 degrees in {axis}, then hold.'),
                   (3, axis + '_return', 'Return to neutral, then release grip.')]
    stages += [(12, 'clutch', 'Move 5 cm with grip held; release; return hand; regrip and move again.'),
               (10, 'gripper', 'Keep hand still; squeeze and release index trigger twice.'),
               (15, 'validation', 'Grip held: trace a slow 5 cm square, then a diagonal; keep orientation fixed.'),
               (5, 'final', 'Release grip; hold still.')]
    return [dict(seconds=s, label=f'{side}_{label}', instruction=msg) for s, label, msg in stages]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--side', choices=['left', 'right'], default='right')
    p.add_argument('--output', type=Path, default=Path('calibration_recordings'))
    p.add_argument('--serial', help='D405 serial; required if multiple D405 devices are connected')
    p.add_argument('--monitor', type=int, default=1, help='MSS monitor index (1 = first display)')
    p.add_argument('--region', type=int, nargs=4, metavar=('X', 'Y', 'WIDTH', 'HEIGHT'))
    p.add_argument('--fps', type=int, default=15, help='screen FPS; camera records at 30 FPS')
    p.add_argument('--seconds', type=float, default=0, help='shorten guided session, e.g. 15 for smoke test')
    p.add_argument('--plan-only', action='store_true', help='print motion cues without accessing hardware')
    a = p.parse_args()
    if not 1 <= a.fps <= 60 or a.seconds < 0:
        p.error('fps must be 1..60; seconds must be >= 0')
    if a.region and (a.region[2] <= 0 or a.region[3] <= 0):
        p.error('region dimensions must be positive')
    return a


def run(a):
    stages = protocol(a.side)
    if a.plan_only:
        elapsed = 0
        for stage in stages:
            print(f'{elapsed:3d}s  {stage["label"]}: {stage["instruction"]}')
            elapsed += stage['seconds']
        print(f'Total: {elapsed}s. Use a helper to read cues while wearing the headset.')
        return 0
    modules = {}
    for name in ('numpy', 'cv2', 'mss', 'pyrealsense2', 'rclpy'):
        try:
            modules[name] = importlib.import_module(name)
        except ImportError as exc:
            raise SystemExit(f'Missing {name}: {exc}. See CALIBRATION.md for setup.') from exc
    np, cv2, mss, rs, rclpy = (modules[n] for n in ('numpy', 'cv2', 'mss', 'pyrealsense2', 'rclpy'))
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, Float32
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    out = a.output.resolve() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=False)
    stop = threading.Event()
    errors = queue.Queue()
    ready = {name: threading.Event() for name in ('camera', 'screen', 'ros')}
    counts = Counter()
    metadata = dict(start_utc=datetime.now(timezone.utc).isoformat(), monotonic_origin_ns=time.monotonic_ns(),
                    options={**vars(a), 'output': str(out)}, protocol=stages,
                    timing='Host monotonic reception/capture times, not hardware synchronization.')
    source = Path(__file__).with_name('quest_aloha_sim.py')
    if source.exists():
        shutil.copy2(source, out / source.name)
        metadata['sim_source_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    (out / 'manifest.json').write_text(json.dumps({**metadata, 'status': 'starting'}, indent=2) + '\n')
    def write(handle, row):
        handle.write(json.dumps(row) + '\n')
        handle.flush()
    def guarded(name, fn):
        try:
            fn()
        except Exception as exc:
            errors.put(f'{name}: {type(exc).__name__}: {exc}')
            stop.set()
    def camera():
        devices = [d for d in rs.context().query_devices()
                   if 'D405' in d.get_info(rs.camera_info.name)]
        if a.serial:
            devices = [d for d in devices if d.get_info(rs.camera_info.serial_number) == a.serial]
        if len(devices) != 1:
            raise RuntimeError(f'Expected one selected D405, found {len(devices)}. Connect camera or use --serial.')
        config = rs.config()
        config.enable_device(devices[0].get_info(rs.camera_info.serial_number))
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_record_to_file(str(out / 'd405.bag'))
        pipe = rs.pipeline()
        profile = pipe.start(config)
        try:
            metadata['camera'] = dict(serial=devices[0].get_info(rs.camera_info.serial_number),
                                      depth_scale=profile.get_device().first_depth_sensor().get_depth_scale())
            with (out / 'camera_frames.jsonl').open('w') as f:
                while not stop.is_set():
                    frames = pipe.wait_for_frames(2000)
                    now = time.monotonic_ns()
                    for frame in frames:
                        write(f, dict(host_ns=now, stream=str(frame.get_profile().stream_type()),
                                      frame_number=frame.get_frame_number(), device_ms=frame.get_timestamp(),
                                      timestamp_domain=str(frame.get_frame_timestamp_domain())))
                    counts['camera_framesets'] += 1
                    ready['camera'].set()
        finally:
            pipe.stop()  # Finalizes the SDK bag, including calibration metadata.
    def screen():
        with mss.mss() as capture:
            if a.region:
                region = dict(zip(('left', 'top', 'width', 'height'), a.region))
            else:
                if not 1 <= a.monitor < len(capture.monitors):
                    raise RuntimeError(f'Invalid monitor; available displays: {capture.monitors[1:]}')
                region = capture.monitors[a.monitor]
            metadata['screen_region'] = region
            writer = cv2.VideoWriter(str(out / 'screen.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                                     a.fps, (region['width'], region['height']))
            if not writer.isOpened():
                raise RuntimeError('Cannot open MJPG screen video writer')
            try:
                with (out / 'screen_frames.jsonl').open('w') as f:
                    while not stop.is_set():
                        before = time.monotonic_ns()
                        pixels = np.array(capture.grab(region))[:, :, :3]
                        after = time.monotonic_ns()
                        writer.write(pixels)
                        write(f, dict(frame=counts['screen_frames'], capture_start_ns=before,
                                      capture_end_ns=after, host_ns=(before + after) // 2))
                        counts['screen_frames'] += 1
                        ready['screen'].set()
                        stop.wait(max(0, 1 / a.fps - (time.monotonic_ns() - before) / 1e9))
            finally:
                writer.release()
    def ros():
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('ik_calibration_recorder')
        seen = set()
        required = {f'/quest/{a.side}/{c}' for c in ('pose', 'grip', 'trigger')}
        required |= {f'/aloha_sim/{a.side}/{c}' for c in ('ee_pose', 'ee_target', 'engaged', 'position_error', 'gripper_position')}
        last_seen = {}
        try:
            with (out / 'telemetry.jsonl').open('w') as f:
                def callback(msg, topic):
                    now = time.monotonic_ns()
                    row = dict(host_ns=now, topic=topic)
                    if isinstance(msg, PoseStamped):
                        p, q = msg.pose.position, msg.pose.orientation
                        row.update(source_stamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                                   frame_id=msg.header.frame_id, position=[p.x, p.y, p.z],
                                   orientation_xyzw=[q.x, q.y, q.z, q.w])
                    else:
                        row['value'] = msg.data
                    write(f, row)
                    counts[topic] += 1
                    seen.add(topic)
                    last_seen[topic] = time.monotonic()
                    if required <= seen:
                        ready['ros'].set()
                for side in ('left', 'right'):
                    specs = [('quest', 'pose', PoseStamped), ('quest', 'grip', Float32),
                             ('quest', 'trigger', Float32), ('aloha_sim', 'ee_pose', PoseStamped),
                             ('aloha_sim', 'ee_target', PoseStamped), ('aloha_sim', 'engaged', Bool),
                             ('aloha_sim', 'position_error', Float32), ('aloha_sim', 'gripper_position', Float32)]
                    for prefix, suffix, typ in specs:
                        topic = f'/{prefix}/{side}/{suffix}'
                        node.create_subscription(typ, topic, lambda msg, t=topic: callback(msg, t), qos_profile_sensor_data)
                while not stop.is_set():
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if ready['ros'].is_set():
                        stale = [t for t in required if time.monotonic() - last_seen[t] > 2]
                        if stale:
                            raise RuntimeError(f'Telemetry missing for >2s: {stale}')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    threads = [threading.Thread(target=guarded, args=(name, fn))
               for name, fn in [('camera', camera), ('screen', screen), ('ros', ros)]]
    complete = False
    try:
        for t in threads:
            t.start()
        print(f'Recording to {out}; waiting for camera, screen and selected-arm ROS topics.', flush=True)
        deadline = time.monotonic() + 15
        while not stop.is_set() and not all(e.is_set() for e in ready.values()):
            if time.monotonic() >= deadline:
                errors.put('Readiness timeout: ' + ', '.join(k for k, e in ready.items() if not e.is_set()))
                stop.set()
                break
            stop.wait(0.1)
        with (out / 'events.jsonl').open('w') as f:
            start = time.monotonic()
            for stage in stages:
                if stop.is_set():
                    break
                print(f'[{stage["label"]}] {stage["instruction"]}', flush=True)
                write(f, dict(host_ns=time.monotonic_ns(), **stage))
                duration = stage['seconds']
                if a.seconds:
                    duration = min(duration, max(0, a.seconds - (time.monotonic() - start)))
                stop.wait(duration)
                if a.seconds and time.monotonic() - start >= a.seconds:
                    break
            complete = not stop.is_set()
    finally:
        stop.set()
        for t in threads:
            if t.ident is not None:
                t.join()
        failures = []
        while not errors.empty():
            failures.append(errors.get())
        metadata.update(end_utc=datetime.now(timezone.utc).isoformat(), counts=dict(counts),
                        errors=failures, status='failed' if failures else ('completed' if complete else 'interrupted'))
        (out / 'manifest.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print(f'{metadata["status"]}: {out}', flush=True)
        for failure in failures:
            print(f'ERROR: {failure}', flush=True)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(run(parse_args()))
