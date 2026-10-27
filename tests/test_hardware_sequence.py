"""Named-pose sequence tests. All motor access is mock-only."""
import copy
import math
import numpy as np
import pytest
from webxr_teleop.hardware.core import validate_sequence
from webxr_teleop.hardware.planner import Planner
from test_hardware_planner import setup


def test_sequence_requires_explicit_valid_commissioned_poses():
    p, _, _ = setup()
    validate_sequence(p)
    with pytest.raises(ValueError, match='commissioning'):
        validate_sequence(p, live=True)
    del p['arms']['left']['sleep_rad']
    with pytest.raises(ValueError, match='sleep_rad'):
        validate_sequence(p)
    p, _, _ = setup()
    p['arms']['left']['sleep_rad'][0] = 9
    validate_sequence(p)  # Sleep intentionally bypasses arm joint bounds.
    p['arms']['left']['opening_rad'][0] = 9
    with pytest.raises(ValueError, match='outside joint limits'):
        validate_sequence(p)


@pytest.mark.parametrize('mode', ['opening', 'sleep'])
def test_named_trajectory_without_quest_preserves_gripper_and_limits(monkeypatch, mode):
    p, q, _ = setup()
    for side in q: q[side][0] = -.1
    planner = Planner(p, q)
    clock = [0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic', lambda: clock[0])
    poses = []; done = False
    for i in range(350):
        clock[0] = i * .02
        result = planner.step(dict(t=clock[0],seq=i,epoch=1,mode=mode,home_id=1,measured=q,hands={'left':None,'right':None}))
        q = result['q']; poses.append(q['left'][:6]); done |= result['home_done']
        assert q['left'][6] == .8
    assert done
    np.testing.assert_allclose(q['left'][:6], p['arms']['left'][mode+'_rad'])
    velocity = np.diff(poses, axis=0)/.02
    assert np.max(abs(velocity)) <= math.radians(30 if mode=='opening' else 15)+1e-6
    assert np.max(abs(np.diff(velocity,axis=0)/.02)) <= math.radians(90)+1e-6


def test_invalid_sleep_path_rejected_before_first_target(monkeypatch):
    p,q,_ = setup(); p['arms']['left']['sleep_rad'][0] = 9
    planner=Planner(p,q)
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:0.)
    for ik in planner.iks.values():
        monkeypatch.setattr(ik.planes, 'safe_fraction', lambda start,end: 0.)
    with pytest.raises(ValueError,match='Whole home path'):
        planner.step(dict(t=0.,seq=1,epoch=1,mode='sleep',home_id=1,measured=q,hands={}))


def test_clutch_captures_feedback_once_with_following_lag():
    from webxr_teleop.hardware.core import Guard
    p,q,_=setup(); arm=p['arms']['left']; measured=q['left']
    guard=Guard(arm['lower_rad'],arm['upper_rad']); guard.arm(measured,1)
    guard.frozen=False; guard.goal=measured[:]; guard.goal[0]+=.01
    target=guard.command(dict(epoch=1,seq=1,issued=1.,q=guard.goal,freeze=True),measured,1.)
    assert target==measured and guard.state=='ARMED'
    drifting=measured[:]; drifting[0]+=.001
    assert guard.clutch(drifting)==target


@pytest.mark.parametrize('mode', ['opening', 'sleep'])
def test_sequence_crosses_joint_bounds_but_teleop_cannot(monkeypatch, mode):
    from webxr_teleop.hardware.core import Guard
    p,q,_=setup()
    for side in q:
        p['arms'][side]['upper_rad'][0]=.05
        if mode=='opening': q[side][0]=.12
    planner=Planner(p,q)
    guards={s:Guard(p['arms'][s]['lower_rad'],p['arms'][s]['upper_rad'],sequence=True) for s in q}
    for s,g in guards.items(): g.arm(q[s],1)
    clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    for i in range(180):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode=mode,home_id=1,measured=q,hands={}))
        for s,g in guards.items():
            g.command(dict(epoch=1,seq=i,issued=clock[0],q=result['q'][s],mode=mode),q[s],clock[0])
            assert g.state=='ARMED',g.reason
        q=result['q']
    assert result['home_done']
    for s in q:
        assert q[s][:6]==pytest.approx(p['arms'][s][mode+'_rad'])
    # A teleop command still cannot exceed the configured bounds.
    g=guards['left']; q['left'][0]=.12
    g.command(dict(epoch=1,seq=999,issued=4.,q=q['left'],mode='teleop'),q['left'],4.)
    assert g.state=='HOLD' and 'outside limits' in g.reason


def test_nonsequence_worker_rejects_sleep_limit_bypass():
    from webxr_teleop.hardware.core import Guard
    g=Guard([-.05]*6,[.05]*6);q=[.06]+[0.]*6
    g.arm(q,1)
    g.command(dict(epoch=1,seq=1,issued=0.,q=q,mode='sleep'),q,0.)
    assert g.state=='HOLD'


def test_automatic_worker_startup_has_no_manual_countdown(tmp_path):
    import json
    import sys
    import time
    from webxr_teleop.hardware.run import spawn_worker
    from webxr_teleop.hardware.ipc import receive, send
    p,_,_=setup()
    p['arms']['left']['initial_rad'][0]=.12
    p['arms']['left']['upper_rad'][0]=.05
    path=tmp_path/'profile.json';path.write_text(json.dumps(p))
    start=time.monotonic()
    sock,proc=spawn_worker(path,'left','mock',sys.executable,startup=True,sequence=True)
    messages=[]
    try:
        while time.monotonic()-start<3:
            messages.extend(receive(sock))
            if any(m['type']=='feedback' for m in messages): break
            time.sleep(.01)
        startup=next(m for m in messages if m['type']=='startup')
        assert startup['details']['torque_enabled_once']
        feedback=next(m for m in messages if m['type']=='feedback')
        assert feedback['state']=='HOLD' and feedback['q'][0]==.12
        assert not any(m['type']=='fault' for m in messages)
    finally:
        send(sock,dict(op='shutdown'));proc.wait(timeout=2);sock.close()
