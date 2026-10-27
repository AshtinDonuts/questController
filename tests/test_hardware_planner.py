import copy
import json
import math
import numpy as np
import pytest
from webxr_teleop.hardware.core import load_profile, SIDES
from webxr_teleop.hardware.run import DEFAULT_PROFILE
from webxr_teleop.hardware.planner import Planner


def setup():
    p=load_profile(DEFAULT_PROFILE)
    q={s:p['arms'][s]['initial_rad']+[p['arms'][s]['initial_gripper']] for s in SIDES}
    hands={s:dict(position=[0,1.2,-.4],quaternion=[0,0,0,1],grip=False,trigger=0) for s in SIDES}
    return p,q,hands


def test_home_ignores_controllers_and_freezes_grippers(monkeypatch):
    p,q,h=setup()
    for s in SIDES: q[s][0]=.12
    planner=Planner(p,q)
    clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    poses=[]; done=False
    for i in range(250):
        clock[0]=i*.02
        for s in SIDES:
            h[s]['position']=[i*.05,20,30]
            h[s]['grip']=i%2==0; h[s]['trigger']=i%2
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='home',home_id=1,measured=q,hands=h))
        q=result['q']; poses.append(q['right'][:6]); done|=result['home_done']
        assert q['right'][6]==.8
    assert done
    np.testing.assert_allclose(q['right'][:6],p['arms']['right']['home_rad'],atol=1e-9)
    v=np.diff(poses,axis=0)/.02; a=np.diff(v,axis=0)/.02
    assert np.max(abs(v))<=math.radians(5)+1e-6
    assert np.max(abs(a))<=math.radians(10)+1e-6


def test_home_path_rejected_before_movement(monkeypatch):
    p,q,h=setup(); p['arms']['left']['home_rad'][0]=8
    planner=Planner(p,q)
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:0.)
    with pytest.raises(ValueError,match='Whole home path'):
        planner.step(dict(t=0,seq=1,epoch=1,mode='home',home_id=1,measured=q,hands=h))


def test_hold_epoch_cancels_home(monkeypatch):
    p,q,h=setup(); q['left'][0]=.12
    planner=Planner(p,q); clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    for i in range(20):
        clock[0]=i*.02
        q=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='home',home_id=1,measured=q,hands=h))['q']
    stopped=copy.deepcopy(q)
    for i in range(20,40):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],seq=i,epoch=2,mode='hold',home_id=1,measured=q,hands=h))
        q=result['q']
        assert q==stopped
        assert not result['home_done']
    assert planner.home is None


def test_planner_clutch_does_not_accumulate_feedback_drift(monkeypatch):
    p,q,h=setup(); planner=Planner(p,q); clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    original=copy.deepcopy(q)
    for i in range(10):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='hold',measured=q,hands=h))
        assert result['q']==original
        q['right'][0]+=.00001


@pytest.mark.parametrize('interval', [.0001, .002, .0049])
def test_fresh_jobs_close_together_do_not_fault(monkeypatch, interval):
    p,q,h=setup(); planner=Planner(p,q); clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    job=dict(t=0.,seq=1,epoch=1,mode='hold',measured=q,hands=h)
    first=planner.step(job)
    clock[0]=interval
    result=planner.step(dict(job,t=interval,seq=2))
    assert result['q']==first['q']
    assert all(result['freeze'].values())


@pytest.mark.parametrize('interval', [0., -.001, .151])
def test_invalid_control_intervals_still_fault(monkeypatch, interval):
    p,q,h=setup(); planner=Planner(p,q); clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    job=dict(t=0.,seq=1,epoch=1,mode='hold',measured=q,hands=h)
    planner.step(job)
    clock[0]=interval
    with pytest.raises(ValueError,match='Planner missed control deadline.*interval'):
        planner.step(dict(job,t=interval,seq=2))


@pytest.mark.parametrize('side', SIDES)
def test_stationary_controller_does_not_integrate_motor_sag(monkeypatch, side):
    p,q,h=setup(); planner=Planner(p,q); clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner.step(dict(t=0.,seq=0,epoch=1,mode='align',measured=q,hands=h))
    h[side]['grip']=True
    original=copy.deepcopy(q)
    for i in range(1,251):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='teleop',measured=q,hands=h))
        # The plant follows the command with a fixed load-induced shoulder error.
        # This must not become a new, progressively lower commanded pose.
        q=copy.deepcopy(result['q'])
        q[side][1]-=math.radians(.3)
        for s in SIDES:
            np.testing.assert_allclose(result['q'][s][:6],original[s][:6],atol=1e-8)


def test_trigger_without_side_grip_keeps_arm_frozen(monkeypatch):
    p,q,h=setup(); planner=Planner(p,q); clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner.step(dict(t=0.,seq=0,epoch=1,mode='align',measured=q,hands=h))
    original=copy.deepcopy(q)
    for i in range(1,20):
        clock[0]=i*.02
        h['left']['trigger']=i/20
        h['left']['position'][1]+=.01
        q['left'][1]-=math.radians(.01)
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='teleop',measured=q,hands=h))
        assert result['freeze']['left']
        np.testing.assert_allclose(result['q']['left'][:6],original['left'][:6],atol=1e-8)


def test_controller_upward_motion_still_tracks_with_motor_lag(monkeypatch):
    from webxr_teleop.hardware.planner import put_measured

    p,q,h=setup(); planner=Planner(p,q); clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner.step(dict(t=0.,seq=0,epoch=1,mode='align',measured=q,hands=h))
    h['left']['grip']=True
    initial_z=planner.data.site_xpos[planner.sites['left'],2]
    for i in range(1,151):
        clock[0]=i*.02
        if i==2:h['left']['position'][1]+=.02  # WebXR up maps to robot Z.
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='teleop',measured=q,hands=h))
        q=copy.deepcopy(result['q'])
        q['left'][1]-=math.radians(.3)
    put_measured(planner.model,planner.data,p,q)
    assert planner.data.site_xpos[planner.sites['left'],2]>initial_z+.01


def test_lag_pauses_command_lead_and_resumes_without_reset(monkeypatch):
    p,q,h=setup();p['teleop_catch_up']=True;planner=Planner(p,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner.step(dict(t=0.,seq=0,epoch=1,mode='align',measured=q,hands=h))
    h['left']['grip']=True
    results=[]
    for i in range(1,251):
        clock[0]=i*.02
        h['left']['position'][1]=1.2+min(.10,(i-1)*.001)
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='teleop',measured=q,hands=h))
        json.dumps(result,allow_nan=False)  # Exercise the real IPC encoding boundary.
        results.append(result)
        assert max(abs(a-b) for a,b in zip(result['q']['left'][:6],q['left'][:6]))<=math.radians(2)+1e-9
    assert any(r['diagnostics']['left']['catching_up'] for r in results)
    stalled=np.array(results[-1]['q']['left'])
    for i in range(251,301):
        clock[0]=i*.02;q=result['q']
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='teleop',measured=q,hands=h))
    assert np.max(abs(np.array(result['q']['left'])-stalled))>1e-3
    h['left']['grip']=False;clock[0]+=.02
    result=planner.step(dict(t=clock[0],seq=301,epoch=1,mode='teleop',measured=q,hands=h))
    assert result['freeze']['left']
    np.testing.assert_allclose(result['q']['left'][:6],q['left'][:6])


def test_opening_uses_three_times_original_trajectory_clock(monkeypatch):
    from webxr_teleop.hardware.core import HomeTrajectory
    p,q,h=setup();planner=Planner(p,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    begin=sum([q[s][:6] for s in SIDES],[])
    goal=sum([p['arms'][s]['opening_rad'] for s in SIDES],[])
    original=HomeTrajectory(begin,goal,speed=math.radians(10))
    for i in range(50):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='opening',home_id=1,measured=q,hands=h))
        expected=original.at(clock[0]*3)
        np.testing.assert_allclose(sum([result['q'][s][:6] for s in SIDES],[]),expected)
        q=result['q']


def test_regrip_discards_released_translation_and_rotation(monkeypatch):
    from scipy.spatial.transform import Rotation
    from webxr_teleop.hardware.planner import put_measured
    p,q,h=setup();planner=Planner(p,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    def step(mode='teleop'):
        clock[0]+=.02
        return planner.step(dict(t=clock[0],seq=int(clock[0]*1000),epoch=1,mode=mode,measured=q,hands=h))
    step('align');h['left']['grip']=True;step()
    h['left']['grip']=False;step()
    h['left']['position']=[1.,.6,-1.2]
    h['left']['quaternion']=Rotation.from_euler('xyz',[.5,-.3,1.]).as_quat().tolist()
    step()
    q['left'][0]+=.002  # Measured pose can drift while released.
    put_measured(planner.model,planner.data,p,q)
    expected_p=planner.data.site_xpos[planner.sites['left']].copy()
    expected_r=planner.data.site_xmat[planner.sites['left']].reshape(3,3).copy()
    h['left']['grip']=True;result=step()
    np.testing.assert_allclose(result['diagnostics']['left']['desired_position'],expected_p)
    np.testing.assert_allclose(planner.origin['left']['target_r'],expected_r,atol=1e-12)
    np.testing.assert_allclose(planner.origin['left']['hand'],h['left']['position'])
    np.testing.assert_allclose(result['q']['left'][:6],q['left'][:6],atol=1e-10)


def test_default_teleop_uses_latest_target_without_catchup_pause(monkeypatch):
    p,q,h=setup();planner=Planner(p,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner.step(dict(t=0.,seq=0,epoch=1,mode='align',measured=q,hands=h))
    h['left']['grip']=True
    for i in range(1,151):
        clock[0]=i*.02
        if i==2:h['left']['position'][1]+=.08
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='teleop',measured=q,hands=h))
        diagnostic=result['diagnostics']['left']
        assert 'catching_up' not in diagnostic
        np.testing.assert_allclose(diagnostic['filtered_position'],diagnostic['desired_position'])
        json.dumps(result,allow_nan=False)
    # Fixed measured feedback must no longer stop new commands at two degrees.
    assert max(abs(a-b) for a,b in zip(result['q']['left'][:6],q['left'][:6]))>math.radians(2)


def test_sleep_uses_three_times_original_trajectory_clock(monkeypatch):
    from webxr_teleop.hardware.core import HomeTrajectory
    p,q,h=setup();clock=[0.]
    for side in SIDES:p['arms'][side]['sleep_rad']=q[side][:6];p['arms'][side]['sleep_rad'][0]+=.1
    planner=Planner(p,q)
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    original=HomeTrajectory(sum([q[s][:6] for s in SIDES],[]),sum([p['arms'][s]['sleep_rad'] for s in SIDES],[]))
    for i in range(50):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='sleep',home_id=1,measured=q,hands=h))
        np.testing.assert_allclose(sum([result['q'][s][:6] for s in SIDES],[]),original.at(clock[0]*3))
        q=result['q']
