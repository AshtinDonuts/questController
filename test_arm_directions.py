#!/usr/bin/env python3
"""Deterministic single-arm Cartesian IK test; no Quest or ROS connection."""
import argparse
import time
import mujoco
import mujoco.viewer
import numpy as np
from quest_aloha_sim import SINGLE_MODEL_PATH, SimArm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', type=float, default=0.20, help='metres per direction')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    if not np.isfinite(args.distance) or not 0 < args.distance <= 0.30:
        parser.error('distance must be >0 and <=0.30 metres')
    model = mujoco.MjModel.from_xml_path(str(SINGLE_MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    arm = SimArm(model, data, 'right', single_arm=True)
    home = arm.home_position.copy()
    viewer = None if args.headless else mujoco.viewer.launch_passive(model, data)
    if viewer:
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = model.camera('teleoperator_pov').id
    def segment(start, end, seconds):
        deadline = time.monotonic()
        steps = round(seconds / 0.01)
        for step in range(1, steps + 1):
            if viewer and not viewer.is_running():
                raise KeyboardInterrupt
            alpha = step / steps
            alpha = alpha * alpha * (3 - 2 * alpha)
            arm.target_position = start + alpha * (end - start)
            arm.control(model, data, 0.0)
            data.qvel[:] = 0
            data.time += 0.01
            mujoco.mj_forward(model, data)
            if viewer:
                viewer.sync()
                deadline += 0.01
                time.sleep(max(0, deadline - time.monotonic()))
    try:
        print('Forward = +Y; right = +X; up = +Z. Orientation held fixed.', flush=True)
        segment(home, home, 1)
        for name, direction in [('forward', [0, 1, 0]), ('right', [1, 0, 0]), ('up', [0, 0, 1])]:
            target = home + args.distance * np.array(direction)
            print(f'{name}: command {args.distance * 100:.0f} cm from neutral', flush=True)
            segment(home, target, 3)
            segment(target, target, 2)
            actual = data.site('right/gripper').xpos.copy()
            print(f'  achieved delta xyz (cm): {np.round((actual-home)*100, 2)}; '
                  f'position error: {np.linalg.norm(target-actual)*1000:.1f} mm', flush=True)
            segment(target, home, 3)
            segment(home, home, 1)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer:
            viewer.close()


if __name__ == '__main__':
    main()
