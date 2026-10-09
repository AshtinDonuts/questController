#!/usr/bin/env python3
"""Hold the Quest grip pedal to record each labeled direction motion."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

MOTIONS = [
    ('forward', 'Move FORWARD (away from your torso), keeping orientation steady.'),
    ('right', 'Move RIGHT, keeping orientation steady.'),
    ('up', 'Move UP, keeping orientation steady.'),
    ('tilt_up', 'Tilt the controller so its pointing direction tips UP.'),
    ('turn_right', 'Turn the controller so its pointing direction turns RIGHT.'),
    ('twist_clockwise', 'Twist CLOCKWISE about its pointing axis, looking from the handle toward its tip.'),
]


class GripGate:
    """Fresh release required before each press; stale input discards the trial."""
    def __init__(self):
        self.state = 'release'
        self.grip_time = None
        self.pose_time = None
        self.start = None
        self.rows = []

    def grip(self, value, now):
        if not np.isfinite(value):
            return None
        self.grip_time = now
        if self.state == 'release' and value <= 0.4:
            self.state = 'ready'
        elif self.state == 'ready' and value >= 0.6:
            self.state = 'recording'
            self.start = now
            self.pose_time = now
            self.rows = []
            return 'start'
        elif self.state == 'recording' and value < 0.6:
            self.state = 'release'
            return 'finish'
        return None

    def pose(self, row):
        now = row['host_ns'] / 1e9
        if self.state == 'recording' and now - self.grip_time <= 0.35:
            self.rows.append(row)
            self.pose_time = now
            return True
        return False

    def stale(self, now):
        return self.state == 'recording' and (
            now - self.grip_time > 0.35 or now - self.pose_time > 0.35)


def summarize(rows):
    positions = np.array([s['position'] for s in rows])
    return dict(position=np.median(positions, axis=0).tolist(),
                orientation_xyzw=rows[-1]['orientation_xyzw'],
                position_span=np.ptp(positions, axis=0).tolist(), frames=len(rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=['left', 'right'], default='right')
    parser.add_argument('--include-rotations', action='store_true', help='also record three rotation motions')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'calibration_recordings')
    args = parser.parse_args()
    motions = MOTIONS if args.include_rotations else MOTIONS[:3]
    out = args.output / datetime.now().strftime('axes_%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('quest_axis_recorder')
    raw = (out / 'poses.jsonl').open('w')
    gate = GripGate()
    events, results, errors = [], [], []
    index, trial = 0, 0
    status = 'interrupted'
    cue_given = False

    def event(phase, **fields):
        events.append(dict(host_ns=time.monotonic_ns(), motion=motions[index][0], trial=trial,
                           phase=phase, **fields))

    def prompt():
        print(f'\n[{motions[index][0]}] Return to neutral with pedal RELEASED.\n'
              'Squeeze and HOLD the large middle-finger grip pedal; keep still until MOVE.', flush=True)

    def receive_pose(msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if msg.header.frame_id != 'quest_world_openxr' or not np.all(np.isfinite(values)):
            return
        if not 0.8 < np.linalg.norm(values[3:]) < 1.2:
            return
        row = dict(host_ns=time.monotonic_ns(), frame_id=msg.header.frame_id,
                   source_stamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                   position=values[:3], orientation_xyzw=values[3:])
        if gate.pose(row):
            row.update(motion=motions[index][0], trial=trial)
            raw.write(json.dumps(row) + '\n')
            raw.flush()

    def receive_grip(msg):
        nonlocal index, trial, cue_given
        now = time.monotonic()
        # Detect gaps before a fresh message can conceal a lost stream.
        if gate.stale(now):
            discard()
        action = gate.grip(msg.data, now)
        if action == 'start':
            trial += 1
            cue_given = False
            event('press')
            print('Recording: HOLD STILL for 1 second...', flush=True)
        elif action == 'finish':
            event('release')
            rows = gate.rows
            before = [r for r in rows if r['host_ns'] / 1e9 < gate.start + 0.9]
            after = [r for r in rows if r['host_ns'] / 1e9 > now - 0.5]
            if now - gate.start < 2.5 or len(before) < 5 or len(after) < 5:
                event('rejected', reason='Too short or insufficient endpoint poses')
                print('Trial too short or missing poses. Repeat the same direction.', flush=True)
            else:
                results.append(dict(motion=motions[index][0], instruction=motions[index][1],
                                    trial=trial, before=summarize(before), after=summarize(after)))
                event('accepted')
                print('Captured. Release pedal and return to neutral.', flush=True)
                index += 1
            if index < len(motions):
                prompt()

    def discard():
        event('rejected', reason='Pose or grip stream stale for >0.35 seconds')
        gate.state = 'release'
        gate.rows = []
        print('Tracking/grip data lost: trial discarded. Release pedal and retry.', flush=True)
        prompt()

    node.create_subscription(PoseStamped, f'/quest/{args.side}/pose', receive_pose, qos_profile_sensor_data)
    node.create_subscription(Float32, f'/quest/{args.side}/grip', receive_grip, qos_profile_sensor_data)
    try:
        print(f'Saving to {out}\nNo Enter presses needed. Keep the other hand released.\n'
              'After MOVE, make the motion, hold the endpoint for 1 second, then release the pedal.\n'
              'Stop MuJoCo for this capture: this same grip also engages simulator teleop.\n', flush=True)
        prompt()
        while index < len(motions):
            rclpy.spin_once(node, timeout_sec=0.05)
            if index == len(motions):
                break
            now = time.monotonic()
            if gate.stale(now):
                discard()
            elif gate.state == 'recording' and not cue_given and now - gate.start >= 1:
                cue_given = True
                event('move_cue')
                print('MOVE: ' + motions[index][1] + '\nKeep pedal held; hold endpoint 1 second, then RELEASE.', flush=True)
        status = 'completed'
    except KeyboardInterrupt:
        print('\nStopped; partial recording retained.')
    except Exception as exc:
        errors.append(str(exc))
        status = 'failed'
    finally:
        raw.close()
        node.destroy_node()
        rclpy.shutdown()
        report = dict(side=args.side, status=status, errors=errors, events=events, motions=results,
                      note='Grip-held samples only. Rejected trials remain in raw data with trial IDs. No mapping applied.')
        (out / 'axes.json').write_text(json.dumps(report, indent=2) + '\n')
        print(f'{status}: {out / "axes.json"}')
        for error in errors:
            print(f'ERROR: {error}')
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
