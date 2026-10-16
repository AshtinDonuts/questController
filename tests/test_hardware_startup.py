"""Startup ordering and failure tests exercise the real adapter method, no serial I/O."""
import sys
import time
from types import SimpleNamespace
import numpy as np
import pytest
from webxr_teleop.hardware.bus import LeRobotBus
from webxr_teleop.hardware.core import load_profile
from webxr_teleop.hardware.run import DEFAULT_PROFILE
from webxr_teleop.hardware.planner import Planner
from webxr_teleop.hardware.run import spawn_worker
from webxr_teleop.hardware.ipc import receive, send


class FakeFollower(LeRobotBus):
    def __init__(self, *, mode=4, torque=0, reset=0, partial=False, cancelled=False):
        self.np=np; self.cfg=load_profile(DEFAULT_PROFILE)['arms']['right']
        self.startup_allowed=True; self.startup_attempted=False; self.startup_report=None
        self.raw=None; self.last_q=None; self.events=[]; self.reset=reset; self.partial=partial
        self.position=np.array([10,20,20,30,30,40,50,60,80],dtype=np.int32)
        self.goals=np.full(9,2000)
        self.regs=dict(Model_Number=[1120]*7+[1020]*2,Firmware_Version=[45]*9,
            Torque_Enable=[torque]*9,Operating_Mode=[mode]*8+[5],Drive_Mode=[0]*9,
            Secondary_ID=[255,255,2,255,4,255,255,255,255],Hardware_Error_Status=[0]*9,
            Bus_Watchdog=[0]*9,Goal_Current=[100]*9,Current_Limit=[200]*9,
            Present_Velocity=[0]*9,Profile_Velocity=[0]*9,Profile_Acceleration=[0]*9,Homing_Offset=[0]*9)
        def enable(name,value):
            assert name=='Torque_Enable' and value==1
            self.events.append(('torque',1))
            self.regs[name]=[1]*9 if not partial else [1,0]*4+[1]
            self.position+=reset
        self.bus=SimpleNamespace(write=enable,apply_calibration=lambda raw,names: raw.astype(float))
        # Use a valid calibrated model pose independent of these synthetic raw counts.
        self.decode=lambda raw: ([0.,-.3,.6,0.,-.3,0.,.8],raw.astype(float))

    def register(self,name,names=None): return self.regs[name][:]
    def read_raw(self):
        self.events.append(('read',self.position.tolist()))
        return self.position.copy()
    def write_raw_hold(self,raw):
        self.events.append(('goals',raw.tolist())); self.goals=raw.copy()
    def configure_motion_caps(self): self.events.append(('caps',))
    def watchdog(self): self.events.append(('watchdog',)); self.regs['Bus_Watchdog']=[15]*9


def test_startup_orders_preload_caps_enable_and_verified_hold():
    bus=FakeFollower(); before=bus.position.copy()
    q,health=bus.startup_hold()
    events=[e[0] for e in bus.events]
    assert events.index('goals')<events.index('caps')<events.index('torque')
    assert events.index('watchdog')<events.index('torque')
    assert bus.events.count(('torque',1))==1
    np.testing.assert_array_equal(bus.goals,before)
    assert health['torque'] and bus.startup_report['verified']
    with pytest.raises(RuntimeError,match='once'): bus.startup_hold()
    assert bus.events.count(('torque',1))==1


def test_already_enabled_does_not_cycle_or_reenable():
    bus=FakeFollower(torque=1);bus.startup_hold()
    assert not any(e[0]=='torque' for e in bus.events)
    assert bus.startup_report['verified']


@pytest.mark.parametrize('reset',[4096,-4096,20])
def test_encoder_change_is_held_raw_and_never_accepted(reset):
    bus=FakeFollower(reset=reset)
    with pytest.raises(RuntimeError,match='Encoder reset'):bus.startup_hold()
    np.testing.assert_array_equal(bus.goals,bus.position)
    assert not bus.startup_report['verified']
    assert bus.events.count(('torque',1))==1
    assert bus.regs['Torque_Enable']==[1]*9


def test_position_mode_refused_before_any_write():
    bus=FakeFollower(mode=3)
    with pytest.raises(ValueError,match='extended position'):bus.startup_hold()
    assert not any(e[0] in ('torque','goals','caps') for e in bus.events)


def test_auto_torque_on_goal_update_refused_before_writes():
    bus=FakeFollower();bus.regs['Drive_Mode'][0]=8
    with pytest.raises(ValueError,match='Automatic torque'):bus.startup_hold()
    assert not any(e[0] in ('torque','goals','caps') for e in bus.events)


def test_partial_torque_enable_never_accepts_motion_or_disables_torque():
    bus=FakeFollower(partial=True)
    with pytest.raises(RuntimeError,match='Motor torque'):bus.startup_hold()
    assert bus.events.count(('torque',1))==1
    assert not bus.startup_report['verified']
    np.testing.assert_array_equal(bus.goals,bus.position)


def test_cancel_before_enable_never_enables():
    bus=FakeFollower()
    with pytest.raises(RuntimeError,match='cancelled'):bus.startup_hold(lambda:True)
    assert not any(e[0]=='torque' for e in bus.events)
    assert bus.regs['Torque_Enable']==[0]*9


def test_cancel_after_enable_preserves_torque():
    bus=FakeFollower()
    with pytest.raises(RuntimeError,match='leaving torque enabled'):
        bus.startup_hold(lambda:bus.regs['Torque_Enable']==[1]*9)
    assert bus.regs['Torque_Enable']==[1]*9
    np.testing.assert_array_equal(bus.goals,bus.position)


def test_failed_goal_preload_never_enables():
    bus=FakeFollower()
    def fail(raw):raise ConnectionError('readback mismatch')
    bus.write_raw_hold=fail
    with pytest.raises(ConnectionError):bus.startup_hold()
    assert not any(e[0]=='torque' for e in bus.events)


@pytest.mark.parametrize('side',['left','right'])
def test_single_arm_planner_owns_only_selected_arm(monkeypatch,side):
    profile=load_profile(DEFAULT_PROFILE)
    q={side:profile['arms'][side]['initial_rad']+[.8]}
    planner=Planner(profile,q,(side,))
    assert set(planner.iks)=={side}
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:0.)
    result=planner.step(dict(t=0,seq=1,epoch=0,mode='hold',hands={},measured=q))
    assert set(result['q'])=={side}
    assert result['q']==q


@pytest.mark.parametrize('side',['left','right'])
def test_selected_worker_verifies_startup_before_feedback(side):
    sock,proc=spawn_worker(DEFAULT_PROFILE,side,'mock',sys.executable,startup=True)
    messages=[]
    try:
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            messages.extend(receive(sock))
            if any(m['type']=='feedback' for m in messages): break
            time.sleep(.01)
        startup=next(m for m in messages if m['type']=='startup')
        feedback=next(m for m in messages if m['type']=='feedback')
        assert messages.index(startup)<messages.index(feedback)
        assert startup['details']['verified'] and startup['details']['torque_enabled_once']
        assert feedback['state']=='HOLD' and feedback['hold_confirmed']
        assert feedback['q']==feedback['goal']
    finally:
        send(sock,dict(op='shutdown'));proc.wait(timeout=3);sock.close()
    assert proc.returncode==0


def test_model_limit_override_only_for_explicit_stationary_diagnostic():
    bus=FakeFollower();bus.cfg['lower_rad']=[1.]*6;bus.cfg['upper_rad']=[2.]*6
    with pytest.raises(ValueError,match='joint limits'):bus.startup_hold()
    assert not any(e[0]=='torque' for e in bus.events)
    bus=FakeFollower();bus.cfg['lower_rad']=[1.]*6;bus.cfg['upper_rad']=[2.]*6
    bus.startup_hold(ignore_model_limits=True)
    assert bus.startup_report['verified']
    assert bus.events.count(('torque',1))==1


def test_stationary_watchdog_reuse_never_accepts_fault():
    bus=FakeFollower();bus.regs['Bus_Watchdog']=[15]*9
    with pytest.raises(ValueError,match='watchdog'):bus.inspect()
    bus.reuse_stationary_watchdog=True
    bus.inspect()
    bus.regs['Bus_Watchdog']=[255]*9
    with pytest.raises(ValueError,match='watchdog'):bus.inspect()
