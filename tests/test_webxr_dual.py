import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from webxr_teleop.control import Teleop
from webxr_teleop.run import ROOT
from webxr_teleop.server import ControllerState


def hand(t=0, position=(0, 1.2, -.4), grip=False, trigger=0, buttons=None):
    return ControllerState(np.array(position, dtype=float), Rotation.identity().as_quat(),
                           trigger, grip, buttons or {}, t)


@pytest.fixture
def arms():
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'scenes/dual_viperx_scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    pair = {side: Teleop(model, data, side=side) for side in ('right', 'left')}
    for arm in pair.values():
        arm.step(hand(buttons={'x': True}), hand(), 0, 1/30)
        assert arm.calibrated
    return pair


@pytest.mark.parametrize('side', ['left', 'right'])
def test_matching_hand_moves_only_matching_arm(arms, side):
    active = arms[side]
    other = arms['right' if side == 'left' else 'left']
    start = active.actual_pose()[0]
    fixed = other.data.qpos[other.qids].copy()
    for i in range(100):
        t = (i+1)/30
        inputs = {s: hand(t) for s in arms}
        inputs[side] = hand(t, position=(.1*min(i/60, 1), 1.2, -.4), grip=True, trigger=1)
        for arm in arms.values():
            arm.step(inputs['left'], inputs['right'], t, 1/30)
        assert active.ik_solved
        assert active.ik_position_error < .006
        assert active.ik_joint_speed <= active.cfg.joint_speed+1e-7
        np.testing.assert_array_equal(other.data.qpos[other.qids], fixed)
    assert active.ik_ok
    np.testing.assert_allclose(active.actual_pose()[0]-start, [.1, 0, 0], atol=.006)
    assert active.data.ctrl[active.gripper] < other.data.ctrl[other.gripper]


def test_left_loss_holds_left_while_right_continues(arms):
    for arm in arms.values():
        arm.step(hand(.1, grip=True), hand(.1, grip=True), .1, 1/30)
    fixed = arms['left'].data.qpos[arms['left'].qids].copy()
    start = arms['right'].actual_pose()[0]
    for i in range(90):
        t = .2+i/30
        right = hand(t, position=(0, 1.2+.1*min(i/60,1), -.4), grip=True)
        for arm in arms.values():
            arm.step(None, right, t, 1/30)
    np.testing.assert_array_equal(arms['left'].data.qpos[arms['left'].qids], fixed)
    assert not arms['left'].engaged
    assert arms['right'].actual_pose()[0][2]-start[2] > .09


def test_both_home_and_reset(arms):
    for arm in arms.values():
        arm.data.qpos[arm.qids[0]] += .1
    mujoco.mj_forward(arms['right'].model, arms['right'].data)
    for i in range(80):
        t = .1+i/30
        for arm in arms.values():
            arm.step(hand(t, buttons={'y': i == 0}), hand(t), t, 1/30)
    for arm in arms.values():
        np.testing.assert_allclose(arm.data.qpos[arm.qids], arm.home, atol=1e-5)
        arm.reset_tracking()
        assert not arm.calibrated and not arm.engaged
