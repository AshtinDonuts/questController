import math
import mujoco
import numpy as np
import pytest

from webxr_teleop.control import Config, Teleop
from webxr_teleop.planar_limits import PlanarBounds
from webxr_teleop.run import ROOT
from webxr_teleop.server import ControllerState


def make_arm(scene='single', side='right', bounds=None):
    m=mujoco.MjModel.from_xml_path(str(ROOT/f'scenes/{scene}_viperx_scene.xml'))
    d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
    return Teleop(m,d,Config(planar_bounds=bounds or PlanarBounds()),side=side)


@pytest.mark.parametrize('scene,side', [('single','right'),('dual','left'),('dual','right')])
@pytest.mark.parametrize('target_delta', [[0,0,-.5],[.5,0,0],[-.5,0,0]])
def test_planar_constraints_stop_and_allow_retreat(scene,side,target_delta):
    arm=make_arm(scene,side,PlanarBounds(y_min=-.18,y_max=.18))
    p,r=arm.actual_pose();seen_limit=False
    for _ in range(240):
        previous=arm.data.qpos[arm.qids].copy()
        assert arm.solve(p+target_delta,r)
        seen_limit |= arm.plane_limited
        assert np.min(arm.ik.planes.distances(arm.data)) >= -1e-8
        assert np.max(np.abs(arm.data.qpos[arm.qids]-previous)) <= arm.cfg.joint_speed/30+1e-7
    assert seen_limit
    assert arm.ik_position_error > .03
    # Check actual collision-mesh vertices against the world table independently.
    for geom in arm.ik.planes.geom_ids:
        if arm.model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh=arm.model.geom_dataid[geom];start=arm.model.mesh_vertadr[mesh]
        verts=arm.model.mesh_vert[start:start+arm.model.mesh_vertnum[mesh]]
        points=verts@arm.data.geom_xmat[geom].reshape(3,3).T+arm.data.geom_xpos[geom]
        assert np.min(points[:,2])>=.005-1e-7
    for _ in range(360):
        assert arm.solve(p,r)
    assert arm.ik_position_error < .005


def test_planes_are_fixed_to_base_not_wrist_and_ignore_other_geometry():
    arm=make_arm('dual','left')
    planes=arm.ik.planes
    normals,offsets=planes.normals.copy(),planes.offsets.copy()
    base=arm.model.body('left/base_link').id
    np.testing.assert_allclose(normals[0],arm.data.xmat[base].reshape(3,3)[:,0])
    np.testing.assert_allclose(normals[-1],[0,0,1])
    arm.data.qpos[arm.qids[-1]]+=1.0
    mujoco.mj_forward(arm.model,arm.data)
    np.testing.assert_array_equal(planes.normals,normals)
    np.testing.assert_array_equal(planes.offsets,offsets)
    for g in planes.geom_ids:
        name=arm.model.body(arm.model.geom_bodyid[g]).name
        assert name.startswith('left/') and name!='left/base_link'
    # Overlapping regions are intentional: no dividing wall between the arms.
    other=Teleop(arm.model,arm.data,side='right')
    shared=np.array([0.,.25,.3])
    assert np.min(planes.normals@shared-planes.offsets)>0
    assert np.min(other.ik.planes.normals@shared-other.ik.planes.offsets)>0


def test_finite_step_guard_rejects_penetration():
    arm=make_arm(bounds=PlanarBounds(y_min=-.18,y_max=.18))
    start=arm.data.qpos.copy();candidate=start.copy()
    candidate[arm.qids[0]]+=1.4
    fraction=arm.ik.planes.safe_fraction(start,candidate)
    assert 0<fraction<1
    arm.data.qpos[:]=start+fraction*(candidate-start)
    mujoco.mj_forward(arm.model,arm.data)
    assert np.min(arm.ik.planes.distances(arm.data))>=-1e-8


def test_homing_cannot_bypass_planes():
    arm=make_arm(bounds=PlanarBounds(y_min=-.18,y_max=.18))
    arm.home[0]+=1.4  # A deliberately unsafe home target.
    arm.start_home()
    for i in range(600):
        arm.step(None,None,i/30,1/30)
        assert np.min(arm.ik.planes.distances(arm.data))>=-1e-8
        if 'home_blocked' in arm.events:
            break
    else:
        pytest.fail('Unsafe home was not blocked')
    assert not arm.homing and not arm.released


def test_gripper_passes_through_finite_step_guard(monkeypatch):
    arm=make_arm()
    before=arm.data.qpos[arm.fingers].copy()
    # Exercise the trigger path when the swept-geometry check disallows expansion.
    monkeypatch.setattr(arm.ik.planes,'safe_fraction',lambda start,candidate: 0.)
    hand=ControllerState(np.array([0,1,-.4]),np.array([0.,0,0,1]),0.,False,{},0.)
    arm.step(None,hand,0.,1/30)
    np.testing.assert_array_equal(arm.data.qpos[arm.fingers],before)
    assert arm.plane_limited


def test_invalid_bounds_fail_at_startup():
    with pytest.raises(ValueError):
        PlanarBounds(x_min=1,x_max=0)
    with pytest.raises(ValueError,match='Initial arm'):
        make_arm(bounds=PlanarBounds(y_min=-.02,y_max=.02))


def test_plane_jacobians_match_finite_difference():
    arm=make_arm()
    planes=arm.ik.planes
    # Bring each plane near its closest box corner to activate all six directions.
    planes.offsets += np.min(planes.distances(arm.data),axis=0)-.02
    cfg=arm.ik.configuration
    cfg.update(arm.data.qpos)
    constraint=planes.compute_qp_inequalities(cfg,1/30)
    q=arm.data.qpos.copy();epsilon=1e-7
    for dof in arm.ik.dofs:
        shifted=q.copy();shifted[dof]+=epsilon
        cfg.update(shifted)
        after=planes.compute_qp_inequalities(cfg,1/30)
        assert after.h.shape==constraint.h.shape
        np.testing.assert_allclose((after.h-constraint.h)/epsilon,
                                   -.8*constraint.G[:,dof],atol=2e-7)
    cfg.update(q)
