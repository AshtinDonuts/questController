from types import SimpleNamespace
import pytest
from webxr_teleop.hardware.bus import LeRobotBus


@pytest.mark.parametrize('readback,success',[([0]*9,True),([0]*8+[1],False)])
def test_sleep_torque_off_requires_all_motor_readbacks(readback,success):
    bus=LeRobotBus.__new__(LeRobotBus);writes=[]
    bus.bus=SimpleNamespace(write=lambda name,value:writes.append((name,value)))
    bus.register=lambda name:readback
    if success:
        bus.disable_torque()
        assert bus.torque_release_attempted
    else:
        with pytest.raises(ConnectionError,match='readback'):bus.disable_torque()
        assert bus.torque_release_attempted
    assert writes==[('Torque_Enable',0)]


@pytest.mark.parametrize('invalid',['epoch','expired','pose','mode','sequence','hold'])
def test_invalid_completion_never_disables_torque(invalid):
    from webxr_teleop.hardware.worker import complete_sleep
    from webxr_teleop.hardware.core import Guard
    guard=Guard([-3]*6,[3]*6,sequence=True);guard.arm([0.]*7,1)
    calls=[];bus=SimpleNamespace(disable_torque=lambda:calls.append(True))
    request=dict(epoch=1,issued=1.);q=[0.]*7;mode='sleep';sequence=True
    if invalid=='epoch':request['epoch']=0
    if invalid=='expired':request['issued']=0.
    if invalid=='pose':q[0]=.1
    if invalid=='mode':mode='teleop'
    if invalid=='sequence':sequence=False
    if invalid=='hold':guard.hold(q,'Stop')
    with pytest.raises(ValueError):complete_sleep(bus,guard,request,q,{'sleep_rad':[0.]*6},mode,1.,sequence)
    assert not calls


def test_successful_completion_remains_readable_and_rejects_position_writes():
    from webxr_teleop.hardware.bus import MockBus
    from webxr_teleop.hardware.core import Guard, load_profile
    from webxr_teleop.hardware.run import DEFAULT_PROFILE
    from webxr_teleop.hardware.worker import complete_sleep

    profile=load_profile(DEFAULT_PROFILE)
    cfg=profile['arms']['left']
    q=cfg['sleep_rad'][:]+[.8]
    bus=MockBus(profile,'left')
    bus.write(q)
    guard=Guard(cfg['lower_rad'],cfg['upper_rad'],sequence=True)
    guard.arm(q,1)
    complete_sleep(bus,guard,dict(epoch=1,issued=1.),q,cfg,'sleep',1.,True)
    assert guard.state=='PARKED'
    assert bus.read(torque_off=True)[1]['torque'] is False
    with pytest.raises(RuntimeError,match='disabled'):bus.write(q)


def test_real_adapter_rejects_goal_writes_after_release_attempt():
    # This also covers a failed readback after only some motors disabled.
    bus=LeRobotBus.__new__(LeRobotBus)
    bus.torque_release_attempted=True
    with pytest.raises(RuntimeError,match='disabled'):
        bus.write([0.]*7)


def test_sleep_restores_profiles_before_torque_release():
    from webxr_teleop.hardware.worker import complete_sleep
    from webxr_teleop.hardware.core import Guard
    guard=Guard([-3]*6,[3]*6,sequence=True);guard.arm([0.]*7,1)
    calls=[]
    bus=SimpleNamespace(opening_speed=lambda enabled:calls.append(('profiles',enabled)),
                        disable_torque=lambda:calls.append(('torque',False)))
    complete_sleep(bus,guard,dict(epoch=1,issued=1.),[0.]*7,{'sleep_rad':[0.]*6},'sleep',1.,True)
    assert calls==[('profiles',False),('torque',False)]
    assert guard.state=='PARKED'
