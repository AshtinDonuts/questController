"""Stationary input must brake even with substantial unexecuted pose error."""
import copy
import math
import numpy as np
import pytest
from webxr_teleop.hardware.core import load_profile, SIDES
from webxr_teleop.hardware.run import DEFAULT_PROFILE
from webxr_teleop.hardware.planner import Planner


@pytest.mark.parametrize('side', SIDES)
@pytest.mark.parametrize('catch_up', [False, True])
def test_stop_with_backlog_then_resume(monkeypatch, side, catch_up):
    profile=load_profile(DEFAULT_PROFILE)
    profile['teleop_catch_up']=catch_up
    profile['teleop_joint_speed_deg_s']=13.
    profile['teleop_joint_acceleration_deg_s2']=40.
    q={s:profile['arms'][s]['initial_rad']+[.8] for s in SIDES}
    hands={s:dict(position=[0.,1.2,-.4],quaternion=[0,0,0,1],grip=False,trigger=0.) for s in SIDES}
    planner=Planner(profile,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    def step(mode='teleop'):
        nonlocal q
        clock[0]+=.01
        result=planner.step(dict(t=clock[0],seq=round(clock[0]*100),epoch=1,mode=mode,measured=q,hands=hands))
        q=copy.deepcopy(result['q'])
        return result
    step('align');hands[side]['grip']=True;step()
    history=[q[side][:6]]
    # Fast continuous hand motion builds a backlog against the 13 deg/s cap.
    for _ in range(30):
        hands[side]['position'][1]+=.004
        result=step();history.append(q[side][:6])
    diagnostic=result['diagnostics'][side]
    assert np.linalg.norm(np.array(diagnostic['desired_position'])-planner.data.site_xpos[planner.sites[side]])>.02
    for _ in range(100):
        result=step();history.append(q[side][:6])
    assert result['diagnostics'][side]['stationary_hold']
    stopped=np.array(q[side][:6])
    # Continue with small input jitter and independent aperture changes.
    for i in range(200):
        hands[side]['position'][0]=(-1)**i*.0001
        hands[side]['trigger']=.5
        result=step();history.append(q[side][:6])
        np.testing.assert_allclose(q[side][:6],stopped,atol=1e-8)
    assert q[side][6]<.8
    velocity=np.diff(history,axis=0)/.01
    assert np.max(np.abs(velocity))<=math.radians(13)+1e-7
    assert np.max(np.abs(np.diff(velocity,axis=0)/.01))<=math.radians(40)+1e-6
    # Old target error was discarded: a small new motion starts near the hold.
    for _ in range(30):
        hands[side]['position'][1]+=.001
        result=step()
    assert not result['diagnostics'][side]['controller_stationary']
    assert np.max(np.abs(np.array(q[side][:6])-stopped))>1e-3
    assert result['diagnostics'][side]['ik_position_error_m']<.03
