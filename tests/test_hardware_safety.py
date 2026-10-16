"""No tests in this file open physical serial ports."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest
from webxr_teleop.hardware.core import Guard, HomeTrajectory, PERIOD, load_profile
from webxr_teleop.hardware.ipc import receive, send
from webxr_teleop.hardware.run import DEFAULT_PROFILE, spawn_worker


def guard():
    g=Guard([-3]*6,[3]*6); g.arm([0.]*7,1); g.last_command=0.
    return g


def command(t=.02,seq=1,q=None,**kw):
    return dict(epoch=1,seq=seq,issued=t,q=q or [0.]*7,**kw)


def test_quintic_speed_acceleration_and_sync():
    start=[0.]*12; end=[.2,-.7,.01,0,0,0]*2
    h=HomeTrajectory(start,end)
    t=np.linspace(0,h.duration,10001)
    q=np.array([h.at(x) for x in t]); v=np.gradient(q,t,axis=0); a=np.gradient(v,t,axis=0)
    assert np.max(abs(v)) <= math.radians(5)+1e-6
    assert np.max(abs(a)) <= math.radians(10)+1e-6
    np.testing.assert_allclose(q[0],start); np.testing.assert_allclose(q[-1],end)
    assert np.max(abs(v[[0,-1]])) < 1e-6
    assert np.max(abs(a[[0,-1]])) < 1e-4
    np.testing.assert_allclose(q[:,0]/end[0],q[:,1]/end[1])


@pytest.mark.parametrize('change',[
    dict(issued=-1),dict(epoch=0),dict(seq=0),dict(q=[float('nan')]*7),
    dict(q=[4.]*6+[0]),dict(q=[0]*6+[2]),
])
def test_reject_bad_commands_and_latch(change):
    g=guard(); g.command(command(),[0]*7,.02)
    m=command(.04,2); m.update(change)
    g.command(m,[0]*7,.04)
    assert g.state=='HOLD'
    assert g.command(command(.06,3),[.2]*7,.06)==[0]*7
    assert g.state=='HOLD'


def test_watchdog_captures_measured_once():
    g=guard(); pose=[.01]*6+[.4]
    assert g.tick(pose,.16)==pose
    assert g.tick([.03]*7,.2)==pose
    assert g.hold([.04]*7,'Another stop')==pose
    with pytest.raises(ValueError): g.arm(pose,1)
    g.arm(pose,2)
    assert g.state=='ARMED'


def test_clutch_hold_does_not_creep():
    g=guard()
    g.command(command(q=[.0001]*6+[0]),[0]*7,.02)
    stopped=[.00005]*6+[0]
    g.command(command(.04,2,freeze=True),stopped,.04)
    result=g.command(command(.06,3,freeze=True),[.0001]*6+[0],.06)
    assert result==stopped
    assert g.state=='ARMED'


def test_profile_requires_commissioning():
    assert load_profile(DEFAULT_PROFILE)['version']==1
    with pytest.raises(ValueError,match='commissioning'):
        load_profile(DEFAULT_PROFILE,live=True)


def worker_feedback(sock, timeout=3):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        for m in receive(sock):
            if m['type']=='fault': raise AssertionError(m)
            if m['type']=='feedback': return m
        time.sleep(.005)
    raise AssertionError('Worker feedback timed out')


def test_real_worker_process_survives_parent_command_stall():
    sock,proc=spawn_worker(DEFAULT_PROFILE,'left','mock',sys.executable)
    try:
        f=worker_feedback(sock)
        assert f['state']=='HOLD'
        send(sock,dict(op='arm',epoch=1,issued=time.monotonic()))
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            f=worker_feedback(sock)
            if f['state']=='ARMED': break
        assert f['state']=='ARMED'
        time.sleep(.22)
        # Drain accumulated states; final state must reflect the independent watchdog.
        states=receive(sock)
        assert states[-1]['state']=='HOLD'
        assert states[-1]['hold_confirmed']
        assert states[-1]['reason']=='Command watchdog expired'
        assert states[-1]['q']==states[-1]['goal']
    finally:
        send(sock,dict(op='shutdown')); proc.wait(timeout=2); sock.close()


def test_stop_priority_over_queued_reset_and_motion():
    sock,proc=spawn_worker(DEFAULT_PROFILE,'right','mock',sys.executable)
    try:
        f=worker_feedback(sock)
        os.kill(proc.pid,signal.SIGSTOP)
        send(sock,dict(op='hold',reason='Test stop'))
        send(sock,dict(op='arm',epoch=1,issued=time.monotonic()))
        send(sock,dict(op='command',epoch=1,seq=1,issued=time.monotonic(),q=f['q']))
        os.kill(proc.pid,signal.SIGCONT)
        time.sleep(.07)
        frames=receive(sock)
        assert frames[-1]['state']=='HOLD'
        assert frames[-1]['reason']=='Test stop'
    finally:
        os.kill(proc.pid,signal.SIGCONT)
        send(sock,dict(op='shutdown')); proc.wait(timeout=2); sock.close()


def test_adapter_torque_writes_confined_to_startup_and_successful_sleep():
    import ast
    from webxr_teleop.hardware import bus
    source=Path(bus.__file__).read_text()
    tree=ast.parse(source)
    writes=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
            and n.func.attr in ('write','write1ByteTxRx','write4ByteTxRx')]
    # Torque enable, RAM motion caps, temporary opening velocity, and RAM watchdog only.
    assert len(writes)==6
    opening=[f for f in ast.walk(tree) if isinstance(f,ast.FunctionDef) and f.name=='opening_speed']
    opening_writes=[n for f in opening for n in ast.walk(f) if n in writes]
    assert len(opening_writes)==1 and isinstance(opening_writes[0].args[2],ast.Name)
    torque=[n for n in writes if isinstance(n.args[0],ast.Constant) and n.args[0].value=='Torque_Enable']
    assert len(torque)==2
    owners={n.args[1].value:next(f.name for f in ast.walk(tree) if isinstance(f,ast.FunctionDef) and n in list(ast.walk(f))) for n in torque}
    assert owners=={1:'startup_hold',0:'disable_torque'}
    assert any(len(n.args)>2 and isinstance(n.args[2],ast.Constant) and n.args[2].value==98 for n in writes)
    assert "('Profile_Velocity',112,caps)" in source
    assert "('Profile_Acceleration',108,[1]*9)" in source
    assert 'apply_calibration_autocorrect' not in source
    assert 'set_aloha_robot_preset' not in source


def test_real_lerobot_calibration_roundtrip_without_serial_access():
    python=Path('/home/King/miniconda3/envs/lerobot/bin/python')
    if not python.exists(): pytest.skip('Local LeRobot environment unavailable')
    script='''
import json, math
import numpy as np
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from webxr_teleop.hardware.bus import LeRobotBus
from webxr_teleop.hardware.core import JOINTS, MOTORS
from webxr_teleop.hardware.run import DEFAULT_PROFILE
p=json.load(open(DEFAULT_PROFILE))
for side in ('left','right'):
    a=object.__new__(LeRobotBus)
    a.cfg=p['arms'][side];a.last_q=None;a.np=np
    a.command_names=list(JOINTS)+['gripper']
    a.bus=DynamixelMotorsBus(port='OFFLINE_NO_CONNECTION',motors={n:(i+1,'xm540-w270' if i<7 else 'xm430-w350') for i,n in enumerate(MOTORS)})
    a.bus.set_calibration(json.load(open(a.cfg['calibration'])))
    rng=np.random.default_rng(42)
    for i in range(50):
        q=rng.uniform(-.5,.5,7);q[-1]=rng.uniform(.1,.9)
        raw=np.array(a.raw_target(q.tolist()),dtype=np.int32)
        values=a.bus.apply_calibration(raw,a.command_names)
        back=np.r_[np.deg2rad(values[:6]),values[6]/100]
        np.testing.assert_allclose(back,q,atol=.002)
    assert not a.bus.is_connected
print('100 LeRobot conversions passed without connecting a serial port')
'''
    r=subprocess.run([str(python),'-c',script],capture_output=True,text=True,timeout=15)
    assert r.returncode==0,r.stderr


def test_worker_accepts_skipped_planner_frames_without_false_rate_latch():
    g=guard()
    q=[.005]*6+[0]
    assert g.command(command(q=q),[0]*7,.02)==q
    assert g.state=='ARMED'
    g.tick(q,.18)
    assert g.state=='HOLD' and g.reason=='Command watchdog expired'
