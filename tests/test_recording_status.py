import pytest

from webxr_teleop.hardware.recording_status import recording_status, status_markdown


def progress(**kwargs):
    snapshot=dict(phase='recording',saved=3,episode=4,start_ns=10_000_000_000)
    snapshot.update(kwargs.pop('snapshot',{}))
    options=dict(state='ARMED',reason='Aligned',now_ns=25_000_000_000)
    options.update(kwargs)
    return recording_status(dict(task='Pick and place',num_episodes=50,episode_time_s=60),snapshot,**options)


def test_live_progress_and_saved_count_are_separate_from_attempt_number():
    p=progress()
    assert p['title']=='RECORDING'
    assert p['progress_text']=='0:15 / 1:00 · 0:45 left'
    assert p['episode_progress']==.25
    assert p['episode']==5 and p['saved']==3 and p['target_episodes']==50
    assert 'Saved: 3 / 50' in status_markdown(p)


def test_timer_freezes_at_episode_end_while_grippers_open_and_file_saves():
    p=progress(snapshot=dict(phase='saving',end_ns=20_000_000_000),state='RELEASING')
    assert p['title']=='OPENING GRIPPERS'
    assert p['elapsed_s']==10
    assert 'Capture: saving' in status_markdown(p)


def test_incomplete_opening_and_fault_reasons_are_visible_in_terminal():
    p=progress(state='READY',snapshot=dict(phase='idle'),reason='Opening 10-second limit reached')
    assert p['title']=='OPENING INCOMPLETE'
    assert '10-second limit' in p['summary']
    p=progress(state='FAULT',reason='Planner: IK rejected target')
    assert 'IK rejected target' in p['summary']


def test_unprepared_teleop_is_not_reported_as_powered_hold():
    rejection='Recording rejected: release selected side grips'
    p=progress(state='ARMED',aligned=True,prepared=False,
               snapshot=dict(phase='idle'),result=rejection)
    assert p['title']=='READY TO RECORD'
    assert 'current pose' in p['hint']
    assert rejection in p['summary']
    assert 'Motion held' not in p['summary']


@pytest.mark.parametrize('options,expected',[
    (dict(state='OPENING',returning=True,snapshot=dict(phase='finished')),'RETURNING TO OPENING'),
    (dict(state='PARKING',exiting=True,snapshot=dict(phase='saving')),'SLEEPING'),
    (dict(state='PARKED',exiting=True),'SLEEP COMPLETE'),
    (dict(fault=True,state='PARKING'),'STOPPED / FAULT'),
    (dict(prepared=True,aligned=True,snapshot=dict(phase='idle')),'READY TO RECORD'),
    (dict(prepared=True,snapshot=dict(phase='idle')),'ALIGN CONTROLLERS'),
    (dict(state='HOLD',snapshot=dict(phase='finished',saved=50)),'COLLECTION COMPLETE'),
    (dict(state='HOLD',snapshot=dict(phase='finished',last_status='discarded')),'EPISODE DISCARDED'),
])
def test_stage_priority_and_next_action(options,expected):
    p=progress(**options)
    assert p['title']==expected
    assert p['hint']
