"""All gripper checks are offline: fake registers or mock motor subprocesses."""
import copy
import math
from types import SimpleNamespace
import numpy as np
import pytest
from webxr_teleop.hardware.gripper import (aperture_from_trigger,current_cap,
    validate_calibration,shape_aperture,GRIPPER_SPEED,GRIPPER_ACCELERATION)
from webxr_teleop.hardware.core import Guard,SIDES,load_profile
from webxr_teleop.hardware.run import DEFAULT_PROFILE
from webxr_teleop.hardware.planner import Planner
from webxr_teleop.hardware.bus import LeRobotBus


@pytest.mark.parametrize('trigger,aperture',[(0,1),(.25,.75),(.5,.5),(.75,.25),(1,0),(-.01,1),(1.01,0)])
def test_absolute_trigger_mapping_and_clamping(trigger,aperture):
    assert aperture_from_trigger(trigger)==aperture


@pytest.mark.parametrize('trigger',[None,True,float('nan'),float('inf'),'0'])
def test_missing_invalid_trigger_is_never_treated_as_released(trigger):
    with pytest.raises(ValueError,match='finite tracked'):aperture_from_trigger(trigger)


def registers(goal=200,limit=200,mode=5):
    return dict(Operating_Mode=[mode],Goal_Current=[goal],Current_Limit=[limit])


def test_electrical_current_units_and_preserved_baseline():
    cap=current_cap(registers(),200)
    assert cap['goal_current_ma']==pytest.approx(538)
    assert cap['current_limit_ma']==pytest.approx(538)
    assert current_cap(registers(),200,cap)==cap
    with pytest.raises(ValueError,match='changed'):current_cap(registers(199),200,cap)


@pytest.mark.parametrize('r',[{},registers(0),registers(-1),registers(201),registers(200,0),
    registers(200,1194),registers(mode=3),registers(True),registers(200.0),registers(65535)])
def test_invalid_current_cap_rejected(r):
    with pytest.raises(ValueError,match='current cap'):current_cap(r)


def test_configured_verified_ceiling_rejects_higher_live_readbacks():
    with pytest.raises(ValueError,match='ceiling'):current_cap(registers(201,201),200)
    assert current_cap(registers(100,200),200)['goal_current_counts']==100


def test_live_adapter_requires_cap_and_checks_current_register_readbacks():
    bus=LeRobotBus.__new__(LeRobotBus);bus.cfg={'gripper_current_cap_counts':200}
    r=registers();reads=[]
    def read(name,names):
        assert names==['gripper'];reads.append(name);return r[name]
    bus.register=read
    with pytest.raises(ValueError,match='no verified electrical'):bus.verify_gripper_current()
    with pytest.raises(ValueError,match='no verified electrical'):bus.write([0.]*6+[1.])
    bus._gripper_current_cap=current_cap(r)
    assert bus.verify_gripper_current()['readback_verified']
    assert set(reads)==set(r)
    r['Goal_Current']=[201]
    with pytest.raises(ValueError):bus.verify_gripper_current()
    r.update(registers());r['Operating_Mode']=[3]
    with pytest.raises(ValueError):bus.verify_gripper_current()


def calibration(start=2472,end=3099,mode='LINEAR'):
    return dict(motor_names=['gripper'],start_pos=[start],end_pos=[end],calib_mode=[mode])


@pytest.mark.parametrize('start,end',[(2472,3099),(3099,2472)])
def test_calibration_endpoints_and_raw_aperture_conversion(start,end):
    c=validate_calibration(calibration(start,end))
    bus=LeRobotBus.__new__(LeRobotBus);bus.np=np;bus.last_q=None;bus.command_names=list(range(7))
    bus.cfg=dict(sign=[1]*6,offset_deg=[0]*6)
    # Same LINEAR conversion as LeRobot, with a fake driver and no serial object.
    bus.bus=SimpleNamespace(revert_calibration=lambda values,names:
        np.array(list(values[:6])+[round(start+values[6]/100*(end-start))],dtype=np.int32))
    assert c==dict(closed_raw=start,open_raw=end)
    assert bus.raw_target([0.]*6+[aperture_from_trigger(0)])[-1]==end
    assert bus.raw_target([0.]*6+[aperture_from_trigger(1)])[-1]==start
    assert bus.raw_target([0.]*6+[aperture_from_trigger(.5)])[-1]==round((start+end)/2)


@pytest.mark.parametrize('c',[{},calibration(end=2472),calibration(mode='DEGREE'),calibration(start=math.nan)])
def test_invalid_aperture_calibration_rejected(c):
    with pytest.raises(ValueError,match='aperture endpoints'):validate_calibration(c)


def test_bounded_aperture_speed_acceleration_and_reversals():
    position=.8;velocity=0.;rng=np.random.default_rng(7)
    for i in range(3000):
        dt=float(rng.uniform(.005,.025))
        target=(1.,0.,.25,.75)[(i//300)%4]
        before=velocity
        position,velocity=shape_aperture(position,velocity,target,dt)
        assert 0<=position<=1
        assert abs(velocity)<=GRIPPER_SPEED+1e-12
        assert abs(velocity-before)<=GRIPPER_ACCELERATION*dt+1e-12


def setup(monkeypatch):
    p=load_profile(DEFAULT_PROFILE)
    q={s:p['arms'][s]['initial_rad']+[p['arms'][s]['initial_gripper']] for s in SIDES}
    hands={s:dict(position=[0.,1.2,-.4],quaternion=[0.,0.,0.,1.],grip=False,trigger=0.,t=0.) for s in SIDES}
    clock=[0.];monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner=Planner(p,q)
    def step(mode='teleop',epoch=1):
        clock[0]+=.02
        for h in hands.values():
            if h is not None:h['t']=clock[0]
        return planner.step(dict(t=clock[0],seq=int(clock[0]*1000),epoch=epoch,mode=mode,measured=q,hands=hands))
    step('align')
    return planner,q,hands,clock,step


@pytest.mark.parametrize('trigger,expected',[(0,1),(.35,.65),(1,0)])
def test_trigger_reaches_absolute_aperture_without_side_grip(monkeypatch,trigger,expected):
    planner,q,h,clock,step=setup(monkeypatch);arm=q['left'][:6]
    for i in range(600):
        h['left']['trigger']=trigger;h['left']['position'][0]+=.01
        r=step();q.update(r['q'])
        assert r['freeze']['left'] and r['gripper_active']
        np.testing.assert_allclose(q['left'][:6],arm,atol=1e-12)
    assert q['left'][6]==pytest.approx(expected,abs=1e-8)


def test_arm_clutch_does_not_reset_aperture_trajectory(monkeypatch):
    planner,q,h,clock,step=setup(monkeypatch);h['left']['trigger']=1
    last=q['left'][6];velocity=0.
    for i in range(200):
        h['left']['grip']=(i//10)%2==1
        # Simulate persistent gripper following error: measured q stays fixed.
        r=step();v=(r['q']['left'][6]-last)/.02
        assert v<0
        assert abs(v)<=GRIPPER_SPEED+1e-10
        assert abs(v-velocity)<=GRIPPER_ACCELERATION*.02+1e-10
        last=r['q']['left'][6];velocity=v
    assert last<.2


@pytest.mark.parametrize('invalid',['missing','expired','trigger'])
def test_planner_lost_input_cannot_open_gripper(monkeypatch,invalid):
    planner,q,h,clock,step=setup(monkeypatch)
    if invalid=='missing':h['left']=None
    elif invalid=='expired':h['left']['t']=-1
    else:h['left']['trigger']=None
    clock[0]+=.02
    with pytest.raises(ValueError):
        planner.step(dict(t=clock[0],seq=2,epoch=1,mode='teleop',measured=q,hands=h))
    assert planner.last['left'][6]==q['left'][6]


def test_arm_freeze_keeps_gripper_authority_but_stop_captures_both():
    g=Guard([-3]*6,[3]*6);g.arm([0.]*6+[.4],1)
    def cmd(seq,q,freeze):
        return dict(epoch=1,seq=seq,issued=seq*.02,q=q,mode='teleop',gripper_active=True,freeze=freeze)
    g.command(cmd(1,[.01]*6+[.39],False),[0.]*6+[.4],.02)
    measured=[.005]*6+[.38]
    g.clutch(measured,gripper_active=True)
    assert g.goal==[.005]*6+[.39]
    assert g.command(cmd(2,[1.]*6+[.37],True),measured,.04)==[.005]*6+[.37]
    assert g.command(cmd(3,[1.]*6+[.36],True),[.006]*6+[.36],.06)==[.005]*6+[.36]
    stopped=[.006]*6+[.35];g.hold(stopped,'Stop')
    assert g.command(cmd(4,[0.]*6+[1.],True),stopped,.08)==stopped


def test_adapter_inspection_and_health_preserve_read_back_current_cap():
    from test_hardware_startup import FakeFollower
    bus=FakeFollower(torque=1)
    before=copy.deepcopy(bus.regs)
    report=bus.inspect()
    assert report['gripper_current_cap']['goal_current_ma']==pytest.approx(269.)
    _,health=bus.read()
    assert health['gripper_current_cap']['goal_current_counts']==100
    assert bus.regs==before  # No electrical settings were written.
    bus.regs['Current_Limit'][-1]=201
    with pytest.raises(ValueError,match='readback changed'):bus.read()


@pytest.mark.parametrize('bad',[0,-1,201,65535])
def test_adapter_preflight_rejects_invalid_goal_current(bad):
    from test_hardware_startup import FakeFollower
    bus=FakeFollower(torque=1);bus.regs['Goal_Current'][-1]=bad
    with pytest.raises(ValueError,match='current cap'):bus.inspect()
    assert not bus.events


@pytest.mark.parametrize('event',['stop','expired','missing_timestamp'])
def test_mock_worker_stop_and_stale_trigger_override_opening(event):
    import sys
    import time
    from webxr_teleop.hardware.run import spawn_worker
    from webxr_teleop.hardware.ipc import send,receive
    sock,proc=spawn_worker(DEFAULT_PROFILE,'left','mock',sys.executable)
    def feedback(predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            for m in receive(sock):
                assert m.get('type')!='fault',m
                if m.get('type')=='feedback' and predicate(m):return m
            time.sleep(.005)
        raise AssertionError('No expected feedback')
    try:
        q=feedback(lambda m:True)['q']
        send(sock,dict(op='arm',epoch=1,issued=time.monotonic()))
        feedback(lambda m:m['state']=='ARMED')
        now=time.monotonic()
        if event=='stop':send(sock,dict(op='hold',reason='Stop has priority'))
        target=q[:6]+[1.]
        send(sock,dict(op='command',epoch=1,seq=1,issued=now,input_t=now,
            controller_t=(now-1 if event=='expired' else None if event=='missing_timestamp' else now),
            mode='teleop',gripper_active=True,freeze=True,q=target))
        f=feedback(lambda m:m['state']=='HOLD' and m['epoch']==1)
        assert f['goal'][6]==q[6]  # Never map stale/lost input to "open".
        time.sleep(.04)
        for m in receive(sock):
            if m.get('type')=='feedback':assert m['goal'][6]==q[6]
    finally:
        if proc.poll() is None:send(sock,dict(op='shutdown'));proc.wait(timeout=3)
        sock.close()


def test_mock_worker_index_trigger_moves_gripper_with_arm_clutched():
    import sys
    import time
    from webxr_teleop.hardware.run import spawn_worker
    from webxr_teleop.hardware.ipc import send,receive
    sock,proc=spawn_worker(DEFAULT_PROFILE,'left','mock',sys.executable)
    def feedback(predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            for m in receive(sock):
                assert m.get('type')!='fault',m
                if m.get('type')=='feedback' and predicate(m):return m
            time.sleep(.005)
        raise AssertionError('No expected feedback')
    try:
        q=feedback(lambda m:True)['q']
        send(sock,dict(op='arm',epoch=1,issued=time.monotonic()))
        feedback(lambda m:m['state']=='ARMED')
        for i in range(1,8):
            now=time.monotonic();target=q[:6]+[q[6]-.00024*i]
            send(sock,dict(op='command',epoch=1,seq=i,issued=now,input_t=now,controller_t=now,
                mode='teleop',gripper_active=True,freeze=True,q=target))
            f=feedback(lambda m:m['goal']==target)
            assert f['state']=='ARMED' and f['q'][:6]==q[:6]
        assert f['goal'][6]<q[6]
    finally:
        if proc.poll() is None:send(sock,dict(op='shutdown'));proc.wait(timeout=3)
        sock.close()


@pytest.mark.parametrize('value',[0,-1,True,math.nan,math.inf,'0.2'])
def test_invalid_configurable_gripper_speed(value):
    from webxr_teleop.hardware.gripper import gripper_speed
    with pytest.raises(ValueError,match='gripper_speed'):gripper_speed({'gripper_speed':value})


@pytest.mark.parametrize('limit',[.05,.18,.3])
def test_planner_configurable_open_close_speed(monkeypatch,limit):
    p=load_profile(DEFAULT_PROFILE);p['gripper_speed']=limit
    q={s:p['arms'][s]['initial_rad']+[.5] for s in SIDES}
    h={s:dict(position=[0.,1.2,-.4],quaternion=[0.,0.,0.,1.],grip=False,trigger=1.) for s in SIDES}
    clock=[0.];monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner=Planner(p,q)
    planner.step(dict(t=0.,seq=0,epoch=1,mode='align',measured=q,hands=h))
    velocities=[];previous=.5;v=0.
    for i in range(1,401):
        clock[0]=i*.02
        if i==201:
            for hand in h.values():hand['trigger']=0.
        r=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='teleop',measured=q,hands=h))
        aperture=r['q']['left'][6];next_v=(aperture-previous)/.02
        assert abs(next_v)<=limit+1e-10
        assert abs(next_v-v)<=planner.gripper_acceleration*.02+1e-10
        velocities.append(next_v);previous=aperture;v=next_v;q=r['q']
    assert min(velocities)==pytest.approx(-limit)
    assert max(velocities)==pytest.approx(limit)


@pytest.mark.parametrize('value',[0,-1,True,math.nan,math.inf,'3'])
def test_invalid_configurable_gripper_acceleration(value):
    from webxr_teleop.hardware.gripper import gripper_acceleration
    with pytest.raises(ValueError,match='gripper_acceleration'):gripper_acceleration({'gripper_acceleration':value})


def test_acceleration_follows_speed_or_explicit_override():
    from webxr_teleop.hardware.gripper import gripper_acceleration
    assert gripper_acceleration({})==pytest.approx(.6)
    assert gripper_acceleration({'gripper_speed':1.})==pytest.approx(1/.3)
    assert gripper_acceleration({'gripper_speed':1.,'gripper_acceleration':2.})==2.


def test_configurable_acceleration_makes_high_speed_effective():
    def stroke(speed,acceleration):
        position=0.;v=0.;peak=0.
        for i in range(1,2001):
            before=v
            position,v=shape_aperture(position,v,1.,.01,speed_limit=speed,acceleration_limit=acceleration)
            assert abs(v-before)<=acceleration*.01+1e-10
            assert 0<=position<=1 and abs(v)<=speed+1e-10
            peak=max(peak,v)
            if position>=.99:return i*.01,peak
        raise AssertionError('Aperture failed to arrive')
    slow=stroke(.18,.6);fast=stroke(1.,1/.3)
    assert fast[0]<slow[0]/2
    assert fast[1]==pytest.approx(1.)
