"""Deterministic delayed-I/O replay: no sockets, serial devices or wall-clock sleeps."""
import math
import sys
from types import SimpleNamespace

import pytest

from webxr_teleop.hardware.core import TTL, load_profile
from webxr_teleop.hardware.motion import controller_source_fresh, expired_planner_sources
from webxr_teleop.hardware.run import DEFAULT_PROFILE


def result(source, *, mode='teleop', freeze=False, gripper=True):
    return dict(mode=mode,input_t=9.99,issued=9.995,controller_t={'left':source},
                freeze={'left':freeze},gripper_active=gripper)


@pytest.mark.parametrize('age_ms',[151.16,153.99])
def test_fresh_job_with_expired_controller_source_is_not_admitted(age_ms):
    # Recorded failures: the job is only 10 ms old, but its source is expired.
    target=result(10-age_ms/1000)
    assert 10-target['input_t']<TTL
    assert expired_planner_sources(target,('left',),10,TTL)['left']==pytest.approx(age_ms)
    # A replacement from the latest recorded hand is eligible, with no TTL reset.
    target['controller_t']['left']=9.9
    assert expired_planner_sources(target,('left',),10,TTL)=={}


@pytest.mark.parametrize('source',[None,True,math.nan,math.inf,10.001,9.849])
def test_invalid_source_is_rejected_even_with_arm_clutched(source):
    assert expired_planner_sources(result(source,freeze=True),('left',),10,TTL)


def test_local_sequences_do_not_depend_on_controller_tracking():
    for mode in ('opening','sleep','hold','align'):
        assert expired_planner_sources(result(None,mode=mode),('left',),10,TTL)=={}
    assert expired_planner_sources(result(None,freeze=True,gripper=False),('left',),10,TTL)=={}


def test_two_arm_result_is_rejected_as_a_unit_if_one_source_expires():
    target=result(9.99)
    target['controller_t']['right']=9.84
    target['freeze']['right']=False
    assert expired_planner_sources(target,('left','right'),10,TTL)=={'right':pytest.approx(160.)}


def replay_worker(monkeypatch, during_read, *, side='left', profile_delay=0., followup=False):
    """An old command arrives before a 30 ms encoder read; another arrives in it."""
    from webxr_teleop.hardware import worker
    clock=SimpleNamespace(t=10.)
    cfg=load_profile(DEFAULT_PROFILE)['arms'][side]
    q=cfg['initial_rad']+[cfg['initial_gripper']]
    old_q=q[:];old_q[0]+=.0001
    new_q=q[:];new_q[0]+=.0002
    old=dict(op='command',epoch=1,seq=1,issued=10.,input_t=9.99,controller_t=9.86,
             admitted_t=10.,mode='teleop',gripper_active=True,freeze=False,q=old_q)
    new=dict(old,seq=2,issued=10.02,input_t=10.01,controller_t=10.01,
             admitted_t=10.025,q=new_q)
    pending=[dict(op='arm',epoch=1,issued=10.),old]
    messages=[];writes=[]
    class Bus:
        def __init__(self,*args,**kwargs):self.reads=0
        def read(self,**kwargs):
            self.reads+=1
            if self.reads==2:
                clock.t+=.03
                if during_read=='replace':pending.append(new)
                elif during_read=='stop':pending.extend([dict(op='hold',reason='Stop in read'),new])
                elif during_read=='release':pending.extend([
                    dict(op='clutch',epoch=1,issued=10.02,grip_generation=1,gripper_active=True),new])
            return q[:],{}
        def write(self,target):writes.append(target[:])
        def configure_motion_caps(self):pass
        def watchdog(self):pass
        def opening_speed(self,*args,**kwargs):pass
        def teleop_speed(self,*args):clock.t+=profile_delay;return {}
        def close(self):pass
    def receive(_):
        batch=pending[:];pending.clear();return batch
    def send(_,message):
        messages.append(message)
        if message.get('type')=='feedback':
            feedback=[m for m in messages if m.get('type')=='feedback']
            if followup and len(feedback)==1:
                pending.append(dict(new,issued=clock.t,input_t=clock.t,controller_t=clock.t))
            else:pending.append(dict(op='shutdown'))
        return True
    def sleep(dt):clock.t+=max(dt,.000001)
    monkeypatch.setattr(worker,'time',SimpleNamespace(monotonic=lambda:clock.t,sleep=sleep))
    monkeypatch.setattr(worker,'receive',receive)
    monkeypatch.setattr(worker,'send',send)
    monkeypatch.setattr(worker,'MockBus',Bus)
    monkeypatch.setattr(worker,'socket',SimpleNamespace(socket=lambda **kw:SimpleNamespace(
        setblocking=lambda _:None,close=lambda:None)))
    monkeypatch.setattr(worker.signal,'signal',lambda *args:None)
    monkeypatch.setattr(sys,'argv',['worker','--fd','123','--parent-pid',str(worker.os.getppid()),
        '--profile',str(DEFAULT_PROFILE),'--side',side,'--mode','mock'])
    worker.main()
    assert not [m for m in messages if m.get('type')=='fault'],messages
    return q,old,new,[m for m in messages if m.get('type')=='feedback'],writes


@pytest.mark.parametrize('side',['left','right'])
def test_latest_target_arriving_during_read_replaces_expiring_target_this_cycle(monkeypatch,side):
    q,old,new,feedback,writes=replay_worker(monkeypatch,'replace',side=side)
    first=feedback[0]
    # Before correction the pre-read snapshot reaches execution at 170 ms age,
    # causing HOLD despite a fresh replacement already waiting in IPC.
    assert not controller_source_fresh(old,first['t'],TTL)
    assert first['state']=='ARMED' and first['goal']==new['q']
    assert old['q'] not in writes  # No stale-target playback.
    assert first['telemetry']['command_seq']==new['seq']
    assert first['telemetry']['feedback_read_ms']==pytest.approx(30.)
    assert first['telemetry']['admission_to_execution_ms']==pytest.approx(5.)
    assert first['telemetry']['controller_age_upper_bound_ms']==pytest.approx(20.)


@pytest.mark.parametrize('during_read',['stop','release'])
@pytest.mark.parametrize('side',['left','right'])
def test_stop_or_release_arriving_during_read_prevents_queued_motion(monkeypatch,during_read,side):
    q,old,new,feedback,writes=replay_worker(monkeypatch,during_read,side=side)
    assert feedback[0]['goal']==q
    assert old['q'] not in writes and new['q'] not in writes
    assert feedback[0]['state']==('HOLD' if during_read=='stop' else 'ARMED')


@pytest.mark.parametrize('side',['left','right'])
def test_no_fresh_replacement_still_latches_and_fresh_packets_do_not_auto_resume(monkeypatch,side):
    q,old,new,feedback,writes=replay_worker(monkeypatch,'none',side=side,followup=True)
    assert feedback[0]['reason']=='Controller target expired at motor execution'
    assert all(f['state']=='HOLD' and f['goal']==q for f in feedback)
    assert old['q'] not in writes and new['q'] not in writes


@pytest.mark.parametrize('side',['left','right'])
def test_profile_transition_cannot_allow_target_to_expire_before_write(monkeypatch,side):
    q,old,new,feedback,writes=replay_worker(monkeypatch,'replace',side=side,profile_delay=.14)
    assert feedback[0]['state']=='HOLD'
    assert feedback[0]['reason']=='Target expired before serial write'
    assert old['q'] not in writes and new['q'] not in writes
