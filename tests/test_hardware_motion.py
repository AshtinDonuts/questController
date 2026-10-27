import math
import pytest
from webxr_teleop.hardware.motion import (motion_settings, motor_profile_counts,
    latest_motion_messages, ClutchBarrier)
from webxr_teleop.hardware.replay_tracking import baseline_planner, compare_case, synthetic_frames
from webxr_teleop.hardware.planner import Planner
from webxr_teleop.hardware.core import load_profile
from webxr_teleop.hardware.run import DEFAULT_PROFILE


@pytest.mark.parametrize('key',['teleop_joint_speed_deg_s','teleop_joint_acceleration_deg_s2'])
@pytest.mark.parametrize('value',[0,-1,True,math.nan,math.inf,'18',181])
def test_invalid_motion_settings(key,value):
    with pytest.raises(ValueError):motion_settings({key:value})


def test_motor_margin_units():
    assert motion_settings({})==(18.,60.)
    assert motor_profile_counts(18,60)==(20,6)


def test_latest_target_and_stop_preserved():
    commands=[dict(op='command',epoch=1,seq=i) for i in range(100)]
    stop=dict(op='hold',reason='Stop')
    assert latest_motion_messages(commands[:40]+[stop]+commands[40:])==[stop,commands[-1]]


def test_release_barrier_rejects_old_motion_even_in_later_batch():
    barrier=ClutchBarrier()
    barrier.release(dict(issued=10.,grip_generation=2))
    old=dict(op='command',mode='teleop',issued=10.02,input_t=9.99,grip_generation=1)
    assert not barrier.accepts(old)
    assert not barrier.accepts(dict(old,grip_generation=3))
    assert barrier.accepts(dict(old,input_t=10.1,grip_generation=3))
    assert not barrier.accepts(dict(old,input_t=10.2,grip_generation=2))
    assert barrier.accepts(dict(mode='sleep'))


def test_two_stroke_replay_reduces_lag_with_bounded_derivatives_and_release_hold():
    p=load_profile(DEFAULT_PROFILE)
    q={'left':p['arms']['left']['initial_rad']+[p['arms']['left']['initial_gripper']]}
    frames=synthetic_frames()
    before=compare_case(baseline_planner(),p,q,frames)
    after=compare_case(Planner,p,q,frames)
    assert before['failure'] is None and after['failure'] is None
    assert after['position_rmse_mm'] < .7*before['position_rmse_mm']
    assert after['rotation_rmse_deg'] < .7*before['rotation_rmse_deg']
    assert after['peak_command_velocity_deg_s'] <= 18.00001
    assert after['peak_command_acceleration_deg_s2'] <= 60.00001
    assert after['release_command_error_rad'] == 0


def test_motor_worker_discards_delayed_pre_release_target():
    import sys
    import time
    from webxr_teleop.hardware.run import spawn_worker
    from webxr_teleop.hardware.ipc import send, receive
    sock,proc=spawn_worker(DEFAULT_PROFILE,'left','mock',sys.executable,sequence=True,startup=True)
    def feedback(predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            for m in receive(sock):
                if m.get('type')=='feedback' and predicate(m):return m
                assert m.get('type')!='fault',m
            time.sleep(.005)
        raise AssertionError('No expected worker feedback')
    try:
        q=feedback(lambda m:True)['q']
        now=time.monotonic();send(sock,dict(op='arm',epoch=1,issued=now))
        feedback(lambda m:m['state']=='ARMED')
        target=q[:];target[0]+=.001
        issued=time.monotonic()
        cmd=dict(op='command',epoch=1,seq=1,issued=issued,input_t=issued,controller_t=issued,
                 mode='teleop',grip_generation=1,freeze=False,q=target)
        send(sock,cmd);feedback(lambda m:m['goal']==target)
        released=time.monotonic();send(sock,dict(op='clutch',epoch=1,issued=released,grip_generation=2))
        held=feedback(lambda m:m['telemetry']['target_age_ms'] is None)['goal']
        delayed=dict(cmd,seq=2,issued=time.monotonic(),q=[x+.01 for x in target])
        send(sock,delayed)
        m=feedback(lambda m:m['t']>delayed['issued'])
        assert m['goal']==held
        send(sock,dict(op='clutch',epoch=1,issued=time.monotonic(),grip_generation=2))
        send(sock,dict(op='arm',epoch=2,issued=time.monotonic()))
        send(sock,dict(delayed,issued=time.monotonic(),seq=3))
        newer=feedback(lambda m:m['epoch']==2)
        assert newer['state']=='ARMED' and newer['goal']==held
    finally:
        if proc.poll() is None:
            send(sock,dict(op='shutdown'));proc.wait(timeout=3)
        sock.close()


def test_reversal_keeps_velocity_and_acceleration_bounded():
    from scipy.spatial.transform import Rotation
    p=load_profile(DEFAULT_PROFILE)
    q={'left':p['arms']['left']['initial_rad']+[p['arms']['left']['initial_gripper']]}
    frames=synthetic_frames()
    for f in frames:
        if .2<=f['t']<3.2:
            u=math.sin(2*math.pi*(f['t']-.2))
            f['hands']['left']['position'][1]=1.2+.02*u
            f['hands']['left']['quaternion']=Rotation.from_euler('y',.1*u).as_quat().tolist()
    result=compare_case(Planner,p,q,frames)
    assert result['failure'] is None
    assert result['peak_command_velocity_deg_s']<=18.00001
    assert result['peak_command_acceleration_deg_s2']<=60.00001
    assert result['release_command_error_rad']==0


def test_identical_encoder_goal_does_not_regenerate_motor_profile():
    import numpy as np
    from webxr_teleop.hardware.bus import LeRobotBus
    bus=LeRobotBus.__new__(LeRobotBus);bus.np=np;bus._gripper_current_cap={'verified':True}
    bus._last_written_raw=np.array([2048]*7)
    bus.raw_target=lambda q:[2048]*7
    # No serial object exists: an identical quantized target must return before I/O.
    assert bus.write([0.]*7) is False
