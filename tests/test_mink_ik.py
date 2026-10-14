"""Recorded singularity regression and hard limits for the production Mink solver."""
from pathlib import Path
import math
import mujoco
import numpy as np
import pytest
import mink

from webxr_teleop.control import Config, Teleop
from webxr_teleop.run import MODEL
from webxr_teleop.replay_ik import load_rows, replay, summarize

FIXTURE = Path(__file__).parent/'fixtures/dual_wrist_flip_targets.json.gz'


@pytest.mark.parametrize('scene,side', [('dual', 'right'), ('single', 'right'), ('single', 'left')])
def test_exact_recorded_targets_no_wrist_flip(scene, side):
    frames = []
    for model, data, frame in replay(load_rows(FIXTURE), scene=scene, single_side=side):
        assert np.all(np.isfinite(data.qpos))
        for state in frame['arms'].values():
            assert state['accepted']
            assert state['measured_joint_speed_rad_s'] <= math.radians(60)+1e-7
        frames.append(frame)
    assert len(frames) == 780
    for metrics in summarize(frames).values():
        whole = metrics['whole_trajectory']
        assert whole['max_position_error_mm'] < 15
        assert whole['max_orientation_error_deg'] < 4
        flip = metrics['13_to_16_seconds']
        assert flip['forearm_roll_excursion_deg'] < 20
        assert flip['wrist_roll_excursion_deg'] < 20
        assert flip['max_position_error_mm'] < 2
        assert flip['max_orientation_error_deg'] < .5


@pytest.fixture
def arm():
    m=mujoco.MjModel.from_xml_path(str(MODEL))
    d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,0)
    mujoco.mj_forward(m,d)
    return Teleop(m,d,Config(joint_speed=math.radians(30)))


@pytest.mark.parametrize('dt', [1/120, 1/30, .05, .2])
def test_unreachable_target_is_bounded_and_can_recover(arm,dt):
    start_p, start_r = arm.actual_pose()
    for _ in range(120):
        previous=arm.data.qpos[arm.qids].copy()
        assert arm.solve(np.array([.8,.8,.8]),start_r,dt)
        assert np.max(np.abs(arm.data.qpos[arm.qids]-previous)) <= arm.cfg.joint_speed*min(dt,.05)+1e-7
        assert np.all(arm.data.qpos[arm.qids] >= arm.bounds[0]-1e-7)
        assert np.all(arm.data.qpos[arm.qids] <= arm.bounds[1]+1e-7)
    assert not arm.ik_ok and arm.ik_solved
    for _ in range(600):
        assert arm.solve(start_p,start_r,1/30)
    assert arm.ik_position_error < .005 and arm.ik_rotation_error < math.radians(5)


@pytest.mark.parametrize('failure', ['no_solution','nan','speed'])
def test_failed_solver_holds_qpos_and_command(arm, monkeypatch, failure):
    def bad_solver(*args, **kwargs):
        if failure == 'no_solution':
            raise mink.NoSolutionFound('daqp')
        return np.full(arm.model.nv, np.nan if failure == 'nan' else 100.)
    monkeypatch.setattr(mink,'solve_ik',bad_solver)
    q, ctrl = arm.data.qpos.copy(),arm.data.ctrl.copy()
    p,r=arm.actual_pose()
    assert not arm.solve(p+np.array([0,0,.1]),r)
    assert not arm.ik_ok and not arm.ik_solved and arm.ik_reason
    np.testing.assert_array_equal(q,arm.data.qpos)
    np.testing.assert_array_equal(ctrl,arm.data.ctrl)


def test_stationary_target_does_not_spin_wrist(arm):
    p,r=arm.actual_pose()
    q=arm.data.qpos.copy()
    for _ in range(300):
        assert arm.solve(p,r)
    np.testing.assert_allclose(q,arm.data.qpos,atol=1e-8)
