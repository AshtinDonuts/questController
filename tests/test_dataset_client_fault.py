from types import SimpleNamespace
from webxr_teleop.datasets import client as module
from webxr_teleop.hardware.recording_status import recording_status


def client(monkeypatch):
    c=module.RecorderClient.__new__(module.RecorderClient)
    c.control=object();c.pending=None;c.timeout=1;c.last_status=0
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
