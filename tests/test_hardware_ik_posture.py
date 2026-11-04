"""Soft posture preferences must improve posture without locking the wrist."""
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from webxr_teleop.hardware.core import load_profile, SIDES
from webxr_teleop.hardware.run import DEFAULT_PROFILE
from webxr_teleop.hardware.planner import Planner


def make_planner(side='left', enabled=True):
    profile=load_profile(DEFAULT_PROFILE)
    if not enabled:
        profile['ik_preferred_posture_cost']=[0.]*6
        profile['ik_joint_damping_cost']=[.02]*6
    q={s:profile['arms'][s]['home_rad']+[.8] for s in SIDES}
    return Planner(profile,q,(side,)),q


@pytest.mark.parametrize('side',SIDES)
def test_forearm_roll_preference_reduces_travel_while_tracking(side):
    results=[]
    for enabled in (False,True):
        planner,q=make_planner(side,enabled)
        ik=planner.iks[side];ids=planner.qids[side];site=planner.sites[side]
        position=planner.data.site_xpos[site].copy()
        rotation=planner.data.site_xmat[site].reshape(3,3).copy()
        path=[planner.data.qpos[ids].copy()]
        for i in range(300):
            target=rotation@Rotation.from_euler('x',min(i*.15,30),degrees=True).as_matrix()
            result=ik.advance(planner.data,position,target,.01)
            assert result.accepted
            path.append(planner.data.qpos[ids].copy())
        velocity=np.diff(path,axis=0)/.01
        assert np.max(np.abs(velocity))<=math.radians(planner.joint_speed)+1e-7
        assert np.max(np.abs(np.diff(velocity,axis=0)/.01))<=math.radians(planner.joint_acceleration)+1e-6
        assert result.position_error<.005
        assert result.rotation_error<math.radians(2)
        results.append(np.abs(np.diff(path,axis=0)[:,3]).sum())
    assert results[1]<.9*results[0]
    assert results[1]>math.radians(1)  # A preference, not a locked joint.


def test_fixed_posture_survives_clutch_reference_and_braking():
    planner,q=make_planner();ik=planner.iks['left'];ids=planner.qids['left']
    reference=ik.preferred_posture.target_q.copy()
    changed=planner.data.qpos.copy();changed[ids[3]]+=.4
    ik.set_reference(changed)
    np.testing.assert_array_equal(ik.preferred_posture.target_q,reference)
    np.testing.assert_array_equal(ik.posture.target_q,changed)
    planner.data.qpos[:]=changed
    import mujoco
    mujoco.mj_forward(planner.model,planner.data)
    site=planner.sites['left'];p=planner.data.site_xpos[site].copy();r=planner.data.site_xmat[site].reshape(3,3).copy()
    for _ in range(20):
        assert ik.advance(planner.data,p,r,.01,stopping=True).accepted
    np.testing.assert_allclose(planner.data.qpos,changed,atol=1e-10)


@pytest.mark.parametrize('key,value',[
    ('ik_preferred_posture_cost',[-1]*6),
    ('ik_preferred_posture_cost',[0]*5),
    ('ik_joint_damping_cost',[math.nan]*6),
    ('ik_joint_damping_cost',[-1]*6),
])
def test_invalid_preferences_fail_before_planning(key,value):
    profile=load_profile(DEFAULT_PROFILE);profile[key]=value
    q={s:profile['arms'][s]['home_rad']+[.8] for s in SIDES}
    with pytest.raises(ValueError):Planner(profile,q)


def test_upright_bias_reduces_home_deviation_with_small_tracking_tradeoff():
    import mujoco
    deviations=[]
    for enabled in (False,True):
        planner,q=make_planner(enabled=enabled);ik=planner.iks['left'];ids=planner.qids['left'];site=planner.sites['left']
        planner.data.qpos[ids[1]]-=.2;planner.data.qpos[ids[2]]+=.2
        ik.set_reference(planner.data.qpos)
        mujoco.mj_forward(planner.model,planner.data)
        p=planner.data.site_xpos[site].copy();r=planner.data.site_xmat[site].reshape(3,3).copy()
        for _ in range(100):
            result=ik.advance(planner.data,p,r,.01)
            assert result.accepted
        deviations.append(np.linalg.norm(planner.data.qpos[ids][1:3]-np.array(q['left'][1:3])))
        assert result.position_error<.001
        assert result.rotation_error<math.radians(.2)
    assert deviations[1]<deviations[0]-.0005


def test_clutch_without_hand_motion_does_not_seek_preferred_posture(monkeypatch):
    planner,q=make_planner()
    q['left'][3]=.4
    hands={'left':dict(position=[0.,1.2,-.4],quaternion=[0,0,0,1],grip=True,trigger=.2),
           'right':dict(quaternion=[0,0,0,1])}
    clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner.step(dict(t=0.,seq=0,epoch=1,mode='align',measured=q,hands=hands))
    for i in range(1,51):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='teleop',measured=q,hands=hands))
        np.testing.assert_allclose(result['q']['left'][:6],q['left'][:6],atol=1e-9)
