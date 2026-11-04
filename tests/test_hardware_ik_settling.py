"""A recorded near-stationary target must settle while IK remains active."""
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from webxr_teleop.hardware.planner import Planner


@pytest.mark.parametrize('dt', [.005, .01, .02])
def test_recorded_target_settles_without_repeated_joint_corrections(dt):
    fixture = json.loads((Path(__file__).parent / 'fixtures' /
                          'hardware_near_stationary_target.json').read_text())
    planner = Planner(fixture['profile'], fixture['q'], ('left',))
    ik = planner.iks['left']
    diagnostic = fixture['diagnostics']
    position = np.array(diagnostic['desired_position'])
    rotation = Rotation.from_quat(diagnostic['desired_quaternion_xyzw']).as_matrix()
    previous_velocity = np.array(diagnostic['requested_joint_velocity_rad_s'])
    ik.motion_limit.previous = previous_velocity.copy()
    path = [planner.data.qpos[ik.qids].copy()]
    for _ in range(round(6 / dt)):
        # Deliberately keep IK active: tiny hand changes must not require the
        # separate stationary-input brake to prevent solver oscillation.
        result = ik.advance(planner.data, position, rotation, dt)
        assert result.accepted, result.reason
        path.append(planner.data.qpos[ik.qids].copy())
    velocity = np.diff(path, axis=0) / dt
    acceleration = np.diff(np.vstack([previous_velocity, velocity]), axis=0) / dt
    assert np.max(np.abs(velocity)) <= ik.joint_speed + 1e-7
    assert np.max(np.abs(acceleration)) <= ik.motion_limit.acceleration + 1e-6
    late_path = np.array(path)[-round(3 / dt):]
    assert np.max(np.ptp(late_path, axis=0)) < np.deg2rad(.01)
    assert result.position_error < .002
    assert result.rotation_error < np.deg2rad(.5)
