"""Dual-arm control authority, references and preflight; no motor connections."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from webxr_teleop.hardware.core import SIDES, TTL, load_profile
from webxr_teleop.hardware.motion import matching_grip_sides, expired_planner_sources
from webxr_teleop.hardware.planner import Planner, put_measured
from webxr_teleop.hardware.run import DEFAULT_PROFILE


@pytest.mark.parametrize('changed',SIDES)
def test_one_clutch_transition_does_not_discard_other_arm_target(changed):
    current={'left':5,'right':7}
    planned=current.copy();planned[changed]-=1
    target=dict(mode='teleop',grip_generation=planned)
    other=next(s for s in SIDES if s!=changed)
    assert matching_grip_sides(target,SIDES,current)==(other,)
    target['grip_generation']=current.copy()
    assert matching_grip_sides(target,SIDES,current)==SIDES
    target['mode']='sleep'
    assert matching_grip_sides(target,SIDES,dict(left=99,right=99))==SIDES


@pytest.mark.parametrize('stale',SIDES)
def test_stale_source_blocks_whole_dual_target_even_with_one_grip_released(stale):
    target=dict(mode='teleop',controller_t=dict(left=9.99,right=9.99),
                freeze=dict(left=True,right=False),gripper_active=True)
    target['controller_t'][stale]=9.84
    assert expired_planner_sources(target,SIDES,10.,TTL)=={stale:pytest.approx(160.)}


@pytest.mark.parametrize('moving',SIDES)
def test_dual_clutch_and_trigger_references_are_independent(monkeypatch,moving):
    p=load_profile(DEFAULT_PROFILE)
    q={s:p['arms'][s]['initial_rad']+[.5] for s in SIDES}
    original=copy.deepcopy(q)
    hands={s:dict(position=[0.,1.2,-.4],quaternion=[0.,0.,0.,1.],grip=False,trigger=.5) for s in SIDES}
    other=next(s for s in SIDES if s!=moving)
    clock=[0.];planner=Planner(p,q)
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    generations=dict(left=0,right=0)
    def step(mode='teleop'):
        clock[0]+=.02
        for h in hands.values():h['t']=clock[0]
        return planner.step(dict(t=clock[0],seq=int(clock[0]*1000),epoch=1,mode=mode,
            hands=hands,measured=q,grip_generation=generations.copy()))
    step('align')
    hands[moving]['grip']=True;generations[moving]+=1
    step()  # Capture the first motion's reference.
    hands[moving]['position'][1]+=.02
    hands[moving]['trigger']=1.;hands[other]['trigger']=0.
    for _ in range(35):q=step()['q']
    assert max(abs(a-b) for a,b in zip(q[moving][:6],original[moving][:6]))>.001
    np.testing.assert_allclose(q[other][:6],original[other][:6],atol=1e-10)
    assert q[moving][6]<.5 and q[other][6]>.5  # Triggers work on both sides, including unclutched arm.
    hands[moving]['grip']=False;generations[moving]+=1
    held=step()['q'];q=held
    hands[other]['grip']=True;generations[other]+=1
    step()
    hands[other]['position'][1]+=.015
    for _ in range(20):
        # Released controller movement must not contaminate its next motion.
        hands[moving]['position'][0]+=.03
        target=step();q=target['q']
        np.testing.assert_allclose(q[moving][:6],held[moving][:6],atol=1e-10)
    assert max(abs(a-b) for a,b in zip(q[other][:6],original[other][:6]))>.001
    before=copy.deepcopy(q)
    put_measured(planner.model,planner.data,p,q)
    measured_eef=planner.data.site_xpos[planner.sites[moving]].copy()
    hands[moving]['grip']=True;generations[moving]+=1
    target=step()
    np.testing.assert_allclose(target['diagnostics'][moving]['desired_position'],measured_eef,atol=1e-10)
    np.testing.assert_allclose(target['q'][moving][:6],before[moving][:6],atol=1e-8)
    np.testing.assert_allclose(planner.origin[moving]['hand'],hands[moving]['position'])


def live_fixture(tmp_path):
    p=load_profile(DEFAULT_PROFILE)
    calibration=tmp_path/'calibration.json';calibration.write_text('{}')
    digest=hashlib.sha256(b'{}').hexdigest()
    for side in SIDES:
        p['arms'][side].update(verified=True,verification_note='unit test only',
            calibration=str(calibration),calibration_sha256=digest)
    return p


def test_dual_preflight_checks_right_commissioning_before_any_connection(tmp_path):
    p=live_fixture(tmp_path);p['arms']['right']['verified']=False
    path=tmp_path/'rig.json';path.write_text(json.dumps(p))
    with pytest.raises(ValueError,match='right: commissioning'):
        load_profile(path,live=True,sides=SIDES,require_motor_watchdog=False)


def test_dual_preflight_rejects_single_arm_display_placement(tmp_path):
    p=live_fixture(tmp_path)
    p['arms']['right']['base_position']=p['arms']['left']['base_position'][:]
    path=tmp_path/'rig.json';path.write_text(json.dumps(p))
    with pytest.raises(ValueError,match='base positions coincide'):
        load_profile(path,live=True,sides=SIDES,require_motor_watchdog=False)
    # The existing single-arm workflow remains valid.
    load_profile(path,live=True,sides=('left',),require_motor_watchdog=False)


def test_dual_json_keeps_twin_and_independent_hardware_profile():
    path=Path(__file__).resolve().parents[1]/'webxr_teleop/hardware/launch_dual.json'
    config=json.loads(path.read_text())
    assert config['side']=='both' and config['sequence'] is True and config['twin'] is True
    assert (path.parent/config['profile']).is_file()
    assert config['teleop_joint_speed']==18. and config['teleop_joint_acceleration']==60.


def test_dual_feedback_arriving_after_loop_start_is_not_falsely_stale(tmp_path):
    """Force the observed race on every IPC drain using actual mock processes."""
    import os
    import subprocess
    import sys
    from webxr_teleop.hardware.run import ROOT
    runner=tmp_path/'snapshot_race.py'
    runner.write_text('''
import time
from webxr_teleop.hardware import run
original=run.receive
def receive(sock):
    messages=original(sock)
    for message in messages:
        if message.get('type')=='feedback':
            # A new feedback packet produced during this supervisor iteration.
            # Both timestamps still use the same host monotonic clock.
            message['t']=time.monotonic()
    return messages
run.receive=receive
if __name__=='__main__':run.main()
''')
    private=tmp_path/'private';private.mkdir(mode=0o700)
    output=tmp_path/'session'
    result=subprocess.run([sys.executable,str(runner),'--mode','mock','--side','both',
        '--sequence','--demo','--seconds','6','--socket',str(private/'control.sock'),
        '--profile',str(DEFAULT_PROFILE),'--output',str(output)],
        env=dict(os.environ,MUJOCO_GL='egl',PYTHONPATH=str(ROOT)),capture_output=True,text=True,timeout=15)
    assert result.returncode==0,result.stdout+result.stderr
    rows=[json.loads(line) for line in (output/'hardware.jsonl').read_text().splitlines()]
    ticks=[r for r in rows if r.get('event')=='tick']
    assert {'OPENING','READY','ARMED'} <= {r['state'] for r in ticks}
    assert not [r for r in ticks if r['state'] in ('FAULT','HOLD')],result.stdout+result.stderr
    for r in ticks:
        assert all(r['t']>=f['t'] for f in r['feedback'].values())
