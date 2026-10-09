#!/usr/bin/env python3
"""Replay accepted grip-held axis trials locally in MuJoCo; no ROS publishing."""
import argparse
import json
from pathlib import Path
import time
import subprocess
import mujoco
import mujoco.viewer
import numpy as np
from quest_aloha_sim import SINGLE_MODEL_PATH, SimArm, quat_xyzw_to_matrix, load_calibration


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording', type=Path)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--calibration', type=Path)
    p.add_argument('--mp4', type=Path, help='save 25 FPS offscreen replay')
    p.add_argument('--translation-distance', type=float, help='normalize each translation trial peak to this distance in metres (replay only)')
    args = p.parse_args()
    if args.translation_distance is not None and (not np.isfinite(args.translation_distance) or args.translation_distance <= 0):
        p.error('translation-distance must be finite and positive')
    if args.calibration:
        load_calibration(args.calibration)
    report = json.loads(args.recording.read_text())
    rows = [json.loads(line) for line in args.recording.with_name('poses.jsonl').read_text().splitlines()]
    model = mujoco.MjModel.from_xml_path(str(SINGLE_MODEL_PATH))
    data = mujoco.MjData(model)
    viewer = None if args.headless else mujoco.viewer.launch_passive(model, data)
    if viewer:
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = model.camera('teleoperator_pov').id
    renderer = mujoco.Renderer(model, height=480, width=640) if args.mp4 else None
    encoder = None
    if args.mp4:
        args.mp4.parent.mkdir(parents=True, exist_ok=True)
        encoder = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
                                    '-pix_fmt', 'rgb24', '-s', '640x480', '-r', '25', '-i', '-',
                                    '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                                    '-movflags', '+faststart', str(args.mp4)], stdin=subprocess.PIPE)
    results = []
    try:
        for motion in report['motions']:
            trial = [r for r in rows if r['trial'] == motion['trial']]
            if not trial:
                raise ValueError(f'No raw samples for trial {motion["trial"]}')
            scale = 1.0
            if args.translation_distance and motion['motion'] in ('forward', 'right', 'up'):
                initial = np.array(trial[0]['position'])
                peak = max(np.linalg.norm(np.array(r['position']) - initial) for r in trial)
                if peak < 1e-9:
                    raise ValueError('Cannot amplify a zero-motion trial')
                scale = args.translation_distance / peak
                trial = [{**r, 'position': (initial + scale * (np.array(r['position']) - initial)).tolist()} for r in trial]
            mujoco.mj_resetDataKeyframe(model, data, 0)
            mujoco.mj_forward(model, data)
            arm = SimArm(model, data, 'right', single_arm=True)
            origin = data.site('right/gripper').xpos.copy()
            peak_target = peak_actual = 0.0
            start = time.monotonic()
            duration = (trial[-1]['host_ns'] - trial[0]['host_ns']) / 1e9
            cursor = 0
            print(f'Replaying {motion["motion"]}, trial {motion["trial"]}, {duration:.2f}s', flush=True)
            for step in range(int(duration / 0.01) + 1):
                elapsed = step * 0.01
                if viewer:
                    if not viewer.is_running():
                        return
                    time.sleep(max(0, start + elapsed - time.monotonic()))
                while cursor + 1 < len(trial) and (trial[cursor + 1]['host_ns'] - trial[0]['host_ns']) / 1e9 <= elapsed:
                    cursor += 1
                row = trial[cursor]
                stamp = (row['host_ns'] - trial[0]['host_ns']) / 1e9
                sample = dict(pose=(np.array(row['position']), quat_xyzw_to_matrix(row['orientation_xyzw'])),
                              pose_time=stamp, grip=1.0, grip_time=stamp)
                arm.update_target(sample, data, elapsed)
                arm.control(model, data, 0.0)
                data.qvel[:] = 0
                data.time = elapsed
                mujoco.mj_forward(model, data)
                peak_target = max(peak_target, float(np.linalg.norm(arm.target_position - origin)))
                peak_actual = max(peak_actual, float(np.linalg.norm(data.site('right/gripper').xpos - origin)))
                if renderer and step % 4 == 0:
                    renderer.update_scene(data, camera='teleoperator_pov')
                    encoder.stdin.write(renderer.render().tobytes())
                if viewer:
                    viewer.sync()
            result = dict(motion=motion['motion'], translation_scale=scale, tracking_rebases=arm.rebases, peak_target_m=peak_target, peak_achieved_m=peak_actual,
                          final_position_error_m=float(np.linalg.norm(arm.target_position-data.site('right/gripper').xpos)))
            results.append(result)
            print(json.dumps(result), flush=True)
    finally:
        if renderer:
            renderer.close()
        if encoder:
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError('MP4 encoder failed')
        if viewer:
            viewer.close()
    path = args.mp4.with_suffix('.json') if args.mp4 else args.recording.with_name('calibrated_replay_results.json' if args.calibration else 'replay_results.json')
    path.write_text(json.dumps(dict(mapping=str(args.calibration) if args.calibration else 'existing uncalibrated mapping', trials=results), indent=2) + '\n')
    print(f'Saved {path}', flush=True)


if __name__ == '__main__':
    main()
