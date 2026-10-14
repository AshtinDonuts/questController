"""Replay recorded EEF targets through the production Mink IK, without Quest input.

Supports a telemetry.jsonl or the compressed regression fixture. Target replay is
open-loop: it preserves the old requested world poses, even if the new IK lags.
It does not reproduce live clutch rebasing, which depends on achieved poses.
"""
import argparse
from dataclasses import asdict
import gzip
import hashlib
from importlib.metadata import version
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .control import Config, Teleop
from .recording import Recording
from .run import ROOT, arm_state


def load_rows(source):
    source = Path(source)
    if source.suffix == '.gz':
        with gzip.open(source, 'rt') as stream:
            return json.load(stream)['frames']
    return [json.loads(line) for line in source.read_text().splitlines()]


def replay(rows, scene='dual', single_side='right', config=None):
    """Yield model, data and measured metrics. Neither input nor source is modified."""
    model = mujoco.MjModel.from_xml_path(str(ROOT / f'scenes/{scene}_viperx_scene.xml'))
    data = mujoco.MjData(model)
    sides = ('right', 'left') if scene == 'dual' else ('right',)
    source_sides = {s: s if scene == 'dual' else single_side for s in sides}
    first_q = np.array(rows[0]['qpos'])
    if scene == 'dual':
        data.qpos[:] = first_q
    else:
        data.qpos[:] = first_q[:8] if single_side == 'right' else first_q[8:]
    mujoco.mj_forward(model, data)
    arms = {s: Teleop(model, data, config, side=s) for s in sides}
    previous_t = None
    for original in rows:
        t = original['elapsed_s']
        dt = 1/30 if previous_t is None else t-previous_t
        previous_t = t
        output = dict(elapsed_s=t, arms={})
        for side, arm in arms.items():
            source_side = source_sides[side]
            source_arm = original['arms'][source_side]
            p = np.array(source_arm['target_position'])
            if scene == 'single':
                p[0] -= .32 if source_side == 'right' else -.32
            r = Rotation.from_quat(source_arm['target_orientation_xyzw']).as_matrix()
            old_q = data.qpos[arm.qids].copy()
            accepted = arm.solve(p, r, dt)
            # Grippers are independent of IK, and follow the source recording.
            offset = 0 if source_side == 'right' else 8
            data.qpos[arm.fingers] = original['qpos'][offset+6:offset+8]
            data.ctrl[arm.gripper] = data.qpos[arm.fingers[0]]
            arm.target_p, arm.target_r = p, r
            measured_speed = float(np.max(np.abs(data.qpos[arm.qids]-old_q))/dt)
            state = arm_state(arm, None)
            state.update(accepted=accepted, measured_joint_speed_rad_s=measured_speed,
                         joint_position=data.qpos[arm.qids].tolist())
            output['arms'][side] = state
        data.time = t
        mujoco.mj_forward(model, data)
        output['qpos'] = data.qpos.tolist()
        yield model, data, output


def summarize(frames):
    result = {}
    for side in frames[0]['arms']:
        result[side] = {}
        for label, subset in [('whole_trajectory', frames),
                              ('13_to_16_seconds', [f for f in frames if 13 <= f['elapsed_s'] <= 16])]:
            if not subset:
                continue
            states = [f['arms'][side] for f in subset]
            q = np.array([s['joint_position'] for s in states])
            result[side][label] = dict(
                max_position_error_mm=max(s['ik_position_error_m'] for s in states)*1000,
                max_orientation_error_deg=float(np.rad2deg(max(s['ik_rotation_error_rad'] for s in states))),
                max_joint_speed_deg_s=float(np.rad2deg(max(s['measured_joint_speed_rad_s'] for s in states))),
                forearm_roll_excursion_deg=float(np.rad2deg(np.ptp(q[:, 3]))),
                wrist_roll_excursion_deg=float(np.rad2deg(np.ptp(q[:, 5]))),
                failed_solves=sum(not s['accepted'] for s in states))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scene', choices=('single', 'dual'), default='dual')
    parser.add_argument('--single-side', choices=('left', 'right'), default='right')
    args = parser.parse_args()
    rows = load_rows(args.source)
    config = Config()
    metadata = dict(source=str(args.source.resolve()),
                    source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
                    scene=args.scene, single_side=args.single_side, config=asdict(config),
                    ik_backend='mink', mink_version=version('mink'), qp_solver='daqp',
                    replay='exact recorded targets; initial posture reference; no live clutch rebasing')
    recording = None
    frames = []
    status = 'failed'
    try:
        for model, data, frame in replay(rows, args.scene, args.single_side, config):
            if recording is None:
                recording = Recording(args.output, model, 30, metadata)
            recording.write(data, frame)
            frames.append(frame)
        status = 'completed'
    finally:
        if recording:
            recording.close(status)
    summary = summarize(frames)
    (args.output / 'metrics.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))
    print(f'MP4: {args.output.resolve() / "session.mp4"}')


if __name__ == '__main__':
    main()
