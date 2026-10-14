"""Quest WebXR -> single or dual ViperX MuJoCo, adapted from StairVLA teleoperation."""
import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import signal
import time

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation

from .control import Config, Teleop
from .recording import Recording
from .frames import draw_eef_frame
from .server import ControllerState, VRServer

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'scenes/single_viperx_scene.xml'
UPSTREAM_COMMIT = 'c128495cf8c9882734d2d8e21ed7db3435d96920'


def demo_input(t):
    """Known WebXR frame: forward -Z, right +X, up +Y; includes clutch and rotation."""
    # Natural grip orientation: red right, green up, blue backward.
    q = Rotation.identity()
    pos = np.array([0., 1.2, -0.4])
    grip = False
    buttons = {}
    label = 'align'
    if 0.2 <= t < 0.4:
        buttons['x'] = True
    for start, direction, name in [(1., [0, 0, -1], 'forward'), (7., [1, 0, 0], 'right'), (13., [0, 1, 0], 'up')]:
        elapsed = t - start
        if 0 <= elapsed < 5.5:
            grip = True
            # Baseline .5s, 2s outbound, .5 hold, 2s return, .5 rest.
            amount = np.clip((elapsed - 0.5) / 2, 0, 1) if elapsed < 3 else np.clip((5-elapsed) / 2, 0, 1)
            amount = amount*amount*(3-2*amount)
            pos += 0.15*amount*np.array(direction)
            label = name
    if 19 <= t < 24.5:
        grip = True
        phase = np.clip((t-19.5)/2, 0, 1) if t < 22 else np.clip((24-t)/2, 0, 1)
        q = Rotation.from_rotvec(np.array([0., 1., 0.])*np.deg2rad(15)*phase) * q
        label = 'yaw'
    right = ControllerState(pos, q.as_quat(), 0.0, grip, {}, t)
    left = ControllerState(np.array([-0.2, 1.2, -0.4]), q.as_quat(), 0.0, False, buttons, t)
    return left, right, label


def arm_state(arm, controller):
    p, r = arm.actual_pose()
    return dict(position=p.tolist(), orientation_xyzw=Rotation.from_matrix(r).as_quat().tolist(),
                target_position=arm.target_p.tolist(),
                target_orientation_xyzw=Rotation.from_matrix(arm.target_r).as_quat().tolist(),
                calibrated=arm.calibrated, tracking=arm.tracking, engaged=arm.engaged,
                ik_ok=arm.ik_ok, clipped=arm.clipped, message=arm.message,
                controller=None if controller is None else dict(position=controller.position.tolist(),
                    quaternion=controller.quaternion.tolist(), grip=controller.squeeze,
                    trigger=controller.trigger, received_t=controller.received_t))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=('single', 'dual'), default='single')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8443)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--demo', action='store_true', help='synthetic controller check; no network or Quest needed')
    parser.add_argument('--seconds', type=float, default=0, help='0 = until Ctrl+C (demo defaults to 26 seconds)')
    parser.add_argument('--scale', type=float, default=1.0, help='robot metres per controller metre')
    parser.add_argument('--rotation-scale', type=float, default=1.0)
    parser.add_argument('--max-linear-speed', type=float, default=0.35, help='metres/sec')
    parser.add_argument('--max-angular-speed', type=float, default=160, help='degrees/sec')
    parser.add_argument('--output', type=Path, help='new session directory; MP4 and telemetry always saved')
    args = parser.parse_args()
    for key in ('scale', 'rotation_scale', 'max_linear_speed', 'max_angular_speed'):
        if not np.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(f'{key} must be finite and positive')
    if not np.isfinite(args.seconds) or args.seconds < 0:
        parser.error('seconds must be finite and >=0')
    config = Config(scale=args.scale, rotation_scale=args.rotation_scale,
                    linear_speed=args.max_linear_speed, angular_speed=np.deg2rad(args.max_angular_speed))
    model_path = MODEL if args.scene == 'single' else ROOT / 'scenes/dual_viperx_scene.xml'
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    teleop = Teleop(model, data, config)
    arms = {'right': teleop}
    if args.scene == 'dual':
        arms['left'] = Teleop(model, data, config, side='left')
    directory = args.output or ROOT / 'teleop_sessions' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    server = None
    viewer = None
    recording = None
    stopped = False
    status = 'completed'
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        if not args.demo:
            server = VRServer(args.host, args.port)
            server.start()
            print(f'Open {server.url} in the Quest browser; select Start Controller Tracking.', flush=True)
        if not args.headless:
            viewer = mujoco.viewer.launch_passive(model, data)
            with viewer.lock():
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = model.camera('teleoperator_pov').id
        recording = Recording(directory, model, 30, dict(config=asdict(config), scene=args.scene,
                              upstream_commit=UPSTREAM_COMMIT, synthetic=args.demo,
                              started=datetime.now().astimezone().isoformat()))
        print(f'MP4: {directory.resolve() / "session.mp4"}', flush=True)
        print('Align: right controller -Z forward (blue +Z backward), release grips, press left X. Each grip moves its matching arm.', flush=True)
        phase = 'idle'
        generation = -1
        start = time.perf_counter()
        last = start
        last_print = 0
        last_haptic = 0
        step = 0
        duration = args.seconds or (26 if args.demo else 0)
        while not stopped and (viewer is None or viewer.is_running()):
            tick = time.perf_counter()
            elapsed = step / 30 if args.demo else tick - start
            if duration and elapsed >= duration:
                break
            dt = 1/30 if args.demo else min(tick-last, 0.05)
            last = tick
            label = 'live'
            if args.demo:
                left, right, label = demo_input(elapsed)
                if args.scene == 'dual':
                    # Independent phases: right only, left only, then both.
                    if 7 <= elapsed < 13:
                        left.position = right.position + np.array([-0.3, 0., 0.])
                        left.squeeze = right.squeeze
                        right.squeeze = False
                        label = 'left_rightward'
                    elif 13 <= elapsed < 25:
                        left.position = right.position + np.array([-0.3, 0., 0.])
                        left.squeeze = right.squeeze
                        label = 'both_' + label
                now = elapsed
            else:
                left, right, frame_generation = server.snapshot()
                now = time.perf_counter()
                if frame_generation != generation:
                    generation = frame_generation
                    for arm in arms.values():
                        arm.reset_tracking()
                    if phase == 'recording':
                        recording.finish_episode(False)
                        phase = 'idle'
            allow_alignment = args.scene == 'single' or (
                left is not None and right is not None and not left.squeeze and not right.squeeze
                and all(0 <= now-c.received_t < config.timeout for c in (left, right))
                and not any(a.homing for a in arms.values()))
            for arm in arms.values():
                arm.step(left, right, now, dt, allow_alignment=allow_alignment)
            events = list(dict.fromkeys(event for arm in arms.values() for event in arm.events))
            if 'home_reached' in events and any(a.homing for a in arms.values()):
                events.remove('home_reached')
            if 'calibrated' in events:
                payload = dict(webxr_to_world_rotation=teleop.basis.tolist(),
                               note='WebXR session alignment; recalibrate after reconnect or recenter.')
                (directory / 'alignment.json').write_text(json.dumps(payload, indent=2) + '\n')
                if server:
                    server.haptic()
            if 'right_b' in events or 'left_y' in events:
                recording.finish_episode(False)
                phase = 'homing' if teleop.homing else 'idle'
                if server:
                    server.haptic(intensity=0.25, duration_ms=120)
            elif 'right_a' in events:
                if phase == 'idle':
                    for arm in arms.values():
                        arm.start_home()
                    phase = 'homing'
                elif phase == 'ready':
                    recording.start_episode()
                    phase = 'recording'
                    if server:
                        server.haptic()
                elif phase == 'recording':
                    recording.finish_episode(True)
                    phase = 'idle'
                    if server:
                        server.haptic()
            if 'home_reached' in events:
                phase = 'ready'
            if not all(a.tracking for a in arms.values()) and phase == 'recording':
                recording.finish_episode(False)
                phase = 'idle'
            data.time += dt
            mujoco.mj_forward(model, data)
            if viewer:
                with viewer.lock():
                    viewer.user_scn.ngeom = 0
                    for arm in arms.values():
                        draw_eef_frame(viewer.user_scn, data, arm.site)
                viewer.sync()
            actual_p, actual_r = teleop.actual_pose()
            row = dict(host_ns=time.monotonic_ns(), elapsed_s=elapsed, label=label, phase=phase,
                       events=events, calibrated=teleop.calibrated, tracking=teleop.tracking,
                       engaged=teleop.engaged, ik_ok=teleop.ik_ok, clipped=teleop.clipped,
                       position=actual_p.tolist(), orientation_xyzw=Rotation.from_matrix(actual_r).as_quat().tolist(),
                       target_position=teleop.target_p.tolist(), target_orientation_xyzw=Rotation.from_matrix(teleop.target_r).as_quat().tolist(),
                       qpos=data.qpos.tolist(), command=data.ctrl.tolist(),
                       controller=None if right is None else dict(position=right.position.tolist(),
                           quaternion=right.quaternion.tolist(), grip=right.squeeze, trigger=right.trigger, received_t=right.received_t))
            row['arms'] = {side: arm_state(arm, left if side == 'left' else right)
                           for side, arm in arms.items()}
            recording.write(data, row)
            if server:
                base_q, ee_q = teleop.frames_in_vr()
                hint = ('Release grips; right blue +Z backward; left X to align' if not teleop.calibrated else
                        {'idle': 'Grip: move | A: home then record | Y: home', 'homing': 'Returning home | B: cancel',
                         'ready': 'A: start episode | grip: adjust', 'recording': 'A: save episode | B: discard'}[phase])
                server.publish(dict(type='status', phase=phase, title=phase, scene=args.scene, arms=row['arms'], tracking=all(a.tracking for a in arms.values()),
                                    deadman=any(a.engaged for a in arms.values()), clipped=any(a.clipped for a in arms.values()), gripper_closed=bool(right and right.trigger >= 0.5),
                                    ik_ok=all(a.ik_ok for a in arms.values()), message=' | '.join(f'{side}: {a.message}' for side, a in arms.items()), hint=hint, record=True,
                                    task='ViperX simulation', episode=len(recording.episodes)+1, target_episodes='∞',
                                    saved=sum(e['saved'] for e in recording.episodes),
                                    elapsed_s=0 if recording.active is None else (recording.frames-recording.active)/30,
                                    robot_frame_q=base_q, gripper_frame_q=ee_q,
                                    left_gripper_frame_q=arms['left'].frames_in_vr()[1] if 'left' in arms else None))
                if any(not a.ik_ok or a.clipped for a in arms.values()) and tick-last_haptic > 1:
                    server.haptic(intensity=0.2)
                    last_haptic = tick
            if elapsed-last_print >= 2:
                print(f'{label} {phase}: tracked={teleop.tracking} grip={teleop.engaged} '
                      f'IK={teleop.ik_ok} ee={np.round(actual_p, 3)}', flush=True)
                last_print = elapsed
            step += 1
            if not args.demo or viewer:
                time.sleep(max(0, 1/30 - (time.perf_counter()-tick)))
    except Exception:
        status = 'failed'
        raise
    finally:
        if server:
            server.stop()
        if viewer:
            viewer.close()
        if recording:
            recording.close(status)
            print(f'Saved: {directory.resolve() / "session.mp4"}', flush=True)


if __name__ == '__main__':
    main()
