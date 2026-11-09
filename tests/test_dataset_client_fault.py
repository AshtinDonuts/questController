from types import SimpleNamespace
from webxr_teleop.datasets import client as module
from webxr_teleop.hardware.recording_status import recording_status


def client(monkeypatch):
    c=module.RecorderClient.__new__(module.RecorderClient)
    c.control=object();c.pending=None;c.timeout=1;c.last_status=0
    c.awaiting_first_status=False
    c.phase='recording';c.status=dict(phase='recording',start_ns=1_000_000_000)
    c.start_ns=1_000_000_000;c.end_ns=None;c.error=None;c.sample_seq=3
    c.process=SimpleNamespace(poll=lambda:None)
    monkeypatch.setattr(module,'send',lambda *a:True)
    monkeypatch.setattr(module,'receive',lambda *a:[])
    monkeypatch.setattr(module.time,'monotonic',lambda:3.)
    monkeypatch.setattr(module.time,'monotonic_ns',lambda:3_000_000_000)
    return c


def test_abort_freezes_display_even_without_recorder_ack(monkeypatch):
    c=client(monkeypatch)
    c.abort('tracking lost')
    c.poll()
    p=recording_status(dict(task='test',episode_time_s=60),c.status,
        state='HOLD',reason='tracking lost',now_ns=54_000_000_000)
    assert p['elapsed_s']==2
    assert c.status['end_ns']==3_000_000_000


def test_startup_grace_ends_on_first_warming_heartbeat(monkeypatch):
    c=client(monkeypatch)
    c.awaiting_first_status=True;c.phase='warming';c.start_ns=None
    c.poll()  # Three seconds of imports/initialization exceeds runtime timeout.
    assert c.error is None
    monkeypatch.setattr(module,'receive',lambda *a:[dict(phase='warming')])
    c.poll()
    assert not c.awaiting_first_status
    monkeypatch.setattr(module,'receive',lambda *a:[])
    monkeypatch.setattr(module.time,'monotonic',lambda:4.1)
    c.poll()
    assert c.error=='Recorder heartbeat expired'


def test_startup_without_heartbeat_is_bounded(monkeypatch):
    c=client(monkeypatch)
    c.awaiting_first_status=True;c.phase='warming';c.start_ns=None
    monkeypatch.setattr(module.time,'monotonic',lambda:10.1)
    c.poll()
    assert c.error=='Recorder startup heartbeat timed out'


def test_blocked_camera_startup_reports_last_known_identity(monkeypatch):
    c=client(monkeypatch)
    c.status=dict(phase='warming', cameras={
        'top':dict(healthy=False, backend='realsense', device='231622302829',
                   stage='checking connection'),
        'right_wrist':dict(healthy=True)})
    c.poll()
    assert 'Recorder heartbeat expired' in c.error
    assert 'top (realsense device 231622302829): checking connection' in c.error
    assert 'right_wrist' not in c.error


def test_process_exit_during_startup_fails_immediately(monkeypatch):
    c=client(monkeypatch)
    c.awaiting_first_status=True;c.phase='warming';c.start_ns=None
    c.process=SimpleNamespace(poll=lambda:1)
    c.poll()
    assert c.error=='Recorder process exited'
    assert c.phase=='failed'


def test_first_failure_survives_later_heartbeat_and_freezes_timer(monkeypatch):
    c=client(monkeypatch);c.error='Recorder transport overflow'
    c.poll()
    assert c.error=='Recorder transport overflow'
    assert c.status['end_ns']==3_000_000_000
    monkeypatch.setattr(module,'receive',lambda *a:[dict(phase='recording',start_ns=c.start_ns)])
    monkeypatch.setattr(module.time,'monotonic_ns',lambda:54_000_000_000)
    c.poll()
    assert c.status['end_ns']==3_000_000_000
    assert c.error=='Recorder transport overflow'


def test_child_failure_boundary_is_used_and_cause_preserved(monkeypatch):
    c=client(monkeypatch)
    monkeypatch.setattr(module,'receive',lambda *a:[dict(phase='failed',start_ns=c.start_ns,
        end_ns=2_000_000_000,error='Required camera stream became stale')])
    c.poll()
    assert c.end_ns==2_000_000_000
    monkeypatch.setattr(module,'receive',lambda *a:[])
    monkeypatch.setattr(module.time,'monotonic',lambda:60.)
    c.poll()
    assert c.error=='Required camera stream became stale'
    assert c.status['end_ns']==2_000_000_000


def test_pause_freezes_immediately_and_old_status_cannot_unfreeze(monkeypatch):
    c=client(monkeypatch)
    c.pause('tracking lost')
    identity=c.pending['request_id']
    monkeypatch.setattr(module,'receive',lambda *a:[dict(phase='recording',start_ns=c.start_ns)])
    c.poll()
    assert c.phase=='pausing'
    assert c.status['pause_ns']==3_000_000_000
    monkeypatch.setattr(module,'receive',lambda *a:[dict(phase='paused',start_ns=c.start_ns,
        pause_ns=3_000_000_000,paused_ns=0,ack=identity)])
    c.poll()
    assert c.phase=='paused' and c.pending is None
    c.request('resume',resume_ns=5_000_000_000)
    identity=c.pending['request_id']
    monkeypatch.setattr(module,'receive',lambda *a:[dict(phase='recording',start_ns=c.start_ns,
        paused_ns=2_000_000_000,ack=identity)])
    c.poll()
    assert c.phase=='recording' and c.pending is None
    assert 'pause_ns' not in c.status
    p=recording_status(dict(task='test',episode_time_s=60),c.status,
        state='ARMED',reason='aligned',now_ns=6_000_000_000)
    assert p['elapsed_s']==3


def test_stop_aborts_paused_episode(monkeypatch):
    c=client(monkeypatch);c.phase='paused'
    c.abort('Local stop command')
    assert c.pending['op']=='finish'
    assert c.pending['status']=='aborted'


def test_tracking_loss_supersedes_pending_resume(monkeypatch):
    c=client(monkeypatch);c.phase='paused'
    c.request('resume',resume_ns=2_000_000_000)
    c.pause('tracking lost again')
    assert c.phase=='pausing'
    assert c.pending['op']=='pause'


def test_long_save_keeps_heartbeat_watchdog_without_transition_timeout(monkeypatch):
    c=client(monkeypatch)
    c.request('finish',status='saved',end_ns=3_000_000_000)
    monkeypatch.setattr(module.time,'monotonic',lambda:30.)
    monkeypatch.setattr(module,'receive',lambda *a:[dict(phase='saving',start_ns=c.start_ns)])
    c.poll()
    assert c.error is None and c.pending is not None
    monkeypatch.setattr(module,'receive',lambda *a:[])
    monkeypatch.setattr(module.time,'monotonic',lambda:31.1)
    c.poll()
    assert c.error=='Recorder heartbeat expired'


def test_post_finish_backpressure_preserves_barrier_and_next_episode(monkeypatch):
    c=client(monkeypatch);c.data=object()
    c.request('finish',status='saved',end_ns=3_000_000_000)
    barrier=c.pending['through_seq']
    monkeypatch.setattr(module,'send',lambda *args:False)
    for _ in range(100):c.sample('robot',side='left',value={})
    assert c.error is None
    assert c.sample_seq==barrier
    assert c.pending['through_seq']==barrier
    messages=[]
    monkeypatch.setattr(module,'send',lambda sock,msg:messages.append(msg) or True)
    c.sample('status',value={})
    assert messages[-1]['seq']==barrier+1
    c.phase='finished';c.pending=None
    c.request('start',start_ns=4_000_000_000)
    c.sample('robot',side='left',value={})
    assert messages[-1]['seq']==barrier+2
    monkeypatch.setattr(module,'send',lambda *args:False)
    c.sample('robot',side='left',value={})
    assert c.error=='Recorder transport overflow'


def test_active_capture_and_pause_transitions_still_fail_on_overflow(monkeypatch):
    for phase in ('starting','recording','pausing','paused','resuming'):
        c=client(monkeypatch);c.data=object();c.phase=phase
        monkeypatch.setattr(module,'send',lambda *args:False)
        c.sample('controllers',hands={})
        assert c.error=='Recorder transport overflow',phase
        assert c.sample_seq==3


def test_inactive_preview_backpressure_is_not_a_capture_fault(monkeypatch):
    for phase in ('warming','idle','saving','finished'):
        c=client(monkeypatch);c.data=object();c.phase=phase
        monkeypatch.setattr(module,'send',lambda *args:False)
        c.sample('status',value={})
        assert c.error is None,phase
        assert c.sample_seq==3
