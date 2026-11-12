"""Full mock motor + synthetic camera lifecycle, with real Quest packets."""
import asyncio
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import socket
import ssl
import subprocess
import time

import pytest
import websockets

QUEST_PYTHON=Path('/home/King/miniconda3/envs/quest_ros/bin/python')
RECORD_PYTHON=Path('/home/King/miniconda3/envs/lerobot/bin/python')
ROOT=Path(__file__).resolve().parents[1]


@contextmanager
def supervisor(tmp_path, side='both', slow_opening=False, process_group=False, slow_planner=False, full_resolution=False, worker_hold_reason=None, worker_fault_code=None, slow_record_start=False, slow_camera_discovery=False):
    if not QUEST_PYTHON.exists() or not RECORD_PYTHON.exists():
        pytest.skip('Quest and recording interpreters required')
    probe=socket.socket();probe.bind(('127.0.0.1',0));port=probe.getsockname()[1];probe.close()
    private=tmp_path/'private';private.mkdir(mode=0o700)
    endpoint=str(private/'control.sock')
    config=json.loads((ROOT/'webxr_teleop/datasets/example_mock.json').read_text())
    config.update(dataset_dir=str(tmp_path/'dataset'),display_data=False)
    if slow_record_start:config['queue_frames']=64
    if full_resolution:
        camera=dict(backend='synthetic',width=640,height=480,fps=30,streams=['rgb','depth'])
        config['cameras']={name:dict(camera) for name in ('left_arm','top')}
    config_path=tmp_path/'recording.json';config_path.write_text(json.dumps(config))
    extra=[]
    if slow_opening:
        profile=json.loads((ROOT/'webxr_teleop/hardware/example_profile.json').read_text())
        profile['arms']['left']['initial_rad'][5]=-2.9
        profile['arms']['left']['opening_rad'][5]=2.9
        profile_path=tmp_path/'profile.json';profile_path.write_text(json.dumps(profile))
        extra=['--profile',str(profile_path)]
    log=(tmp_path/'supervisor.log').open('w')
    entry=['-m','webxr_teleop.hardware.run']
    if slow_planner:
        runner=tmp_path/'slow_planner.py'
        runner.write_text(f'''import sys, time
sys.path.insert(0, {str(ROOT)!r})
from webxr_teleop.hardware import planner, run
original_run = planner.run
def delayed_run(*args):
    time.sleep(1)
    original_run(*args)
planner.run = delayed_run
if __name__ == '__main__':
    run.main()
''')
        entry=[str(runner)]
    if worker_hold_reason is not None:
        runner=tmp_path/'worker_hold.py'
        runner.write_text(f'''import sys
from pathlib import Path
sys.path.insert(0, {str(ROOT)!r})
from webxr_teleop.hardware import run
original_receive = run.receive
marker = Path({str(tmp_path/'inject-worker-hold')!r})
def receive(sock):
    messages = original_receive(sock)
    for message in messages:
        if marker.exists() and message.get('type') == 'feedback' and message.get('state') == 'ARMED':
            marker.unlink()
            if {worker_fault_code!r} is not None:
                messages.append(dict(type='fault',code={worker_fault_code!r},recoverable=True,reason={worker_hold_reason!r}))
            else:
                message.update(state='HOLD', reason={worker_hold_reason!r})
            break
    return messages
run.receive = receive
if __name__ == '__main__':
    run.main()
''')
        entry=[str(runner)]
    recording_python=RECORD_PYTHON
    if slow_record_start:
        recording_python=tmp_path/'slow_recorder'
        recording_python.write_text(f'''#!{RECORD_PYTHON}
import sys, time
sys.path.insert(0, {str(ROOT)!r})
from webxr_teleop.datasets import recorder
original_init = recorder.Episode.__init__
def delayed_init(self, *args, **kwargs):
    time.sleep(.4)
    original_init(self, *args, **kwargs)
recorder.Episode.__init__ = delayed_init
sys.argv = [sys.argv[2], *sys.argv[3:]]
recorder.main()
''')
        recording_python.chmod(0o755)
    if slow_camera_discovery:
        recording_python=tmp_path/'slow_discovery_recorder'
        recording_python.write_text(f'''#!{RECORD_PYTHON}
import sys, time
sys.path.insert(0, {str(ROOT)!r})
from webxr_teleop.datasets import recorder
original_check = recorder.check_realsense_devices
def delayed_check(cameras):
    time.sleep(1.5)
    original_check(cameras)
recorder.check_realsense_devices = delayed_check
sys.argv = [sys.argv[2], *sys.argv[3:]]
recorder.main()
''')
        recording_python.chmod(0o755)
    proc=subprocess.Popen([str(QUEST_PYTHON),*entry,
        '--mode','mock','--sequence','--no-twin','--side',side,'--port',str(port),'--socket',endpoint,
        '--output',str(tmp_path/'session'),'--recording-config',str(config_path),
        '--recording-python',str(recording_python),'--seconds','45']+extra,
        cwd=ROOT,env=dict(os.environ,MUJOCO_GL='egl'),stdout=log,stderr=log,start_new_session=process_group)
    def control(command='status',request_id=None):
        client=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);client.settimeout(2)
        try:
            client.connect(endpoint)
            payload=command if request_id is None else json.dumps(dict(action=command,request_id=request_id))
            client.sendall((payload+'\n').encode())
            return json.loads(client.recv(65536))
        finally:client.close()
    try:
        yield proc,port,control
    finally:
        proc.send_signal(signal.SIGTERM)
        try:proc.wait(timeout=25)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close()
        assert proc.returncode==0,(tmp_path/'supervisor.log').read_text()


async def exercise(port,control,scenario,start=True,absent=None):
    context=ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT);context.check_hostname=False;context.verify_mode=ssl.CERT_NONE
    deadline=time.monotonic()+12
    while True:
        try:
            ws=await websockets.connect(f'wss://127.0.0.1:{port}',ssl=context);break
        except OSError:
            if time.monotonic()>deadline:raise
            await asyncio.sleep(.05)
    state=dict(x=False,a=False,b=False,grip=False,trigger=.2,lost=False,dx=0,absent=absent)
    token=[None]
    async def read():
        async for raw in ws:
            message=json.loads(raw)
            if message['type']=='freshness':token[0]=message['token']
    async def stream():
        seq=0
        while True:
            hand=dict(tracked=True,position=dict(x=state['dx'],y=1.2,z=-.4),
                quaternion=dict(x=0,y=0,z=0,w=1),trigger=state['trigger'],buttons=dict(squeeze=state['grip']))
            left=dict(hand,buttons=dict(squeeze=state['grip'],x=state['x']))
            right=dict(hand,buttons=dict(squeeze=state['grip'],a=state['a'],b=state['b']))
            seq+=1
            await ws.send(json.dumps(dict(type='controllers',reference_epoch=1,sequence=seq,freshness=token[0],
                left=None if state['lost'] or state['absent']=='left' else left,
                right=None if state['absent']=='right' else right)))
            await asyncio.sleep(.015)
    reader=asyncio.create_task(read());writer=asyncio.create_task(stream())
    async def wait(predicate,seconds=10):
        deadline=time.monotonic()+seconds
        while True:
            status=await asyncio.to_thread(control)
            if predicate(status):return status
            assert time.monotonic()<deadline,status
            await asyncio.sleep(.03)
    async def align():
        state['x']=True;await asyncio.sleep(.12);state['x']=False
        await wait(lambda s:s['state']=='ARMED' and 'Aligned' in s['reason'])
        await asyncio.sleep(.4)
    try:
        await wait(lambda s:s['state']=='READY',seconds=15)
        await align()
        if not start:
            await scenario(state,wait,align)
            return
        await asyncio.to_thread(control,'record-start','start-once')
        await wait(lambda s:s['recording']['phase']=='recording')
        # Retrying a start with the same external ID must not start a new file.
        await asyncio.to_thread(control,'record-start','start-once')
        await asyncio.sleep(.5)
        state['grip']=True
        for i in range(12):
            state['dx']=i*.0004
            await asyncio.sleep(.02)
        state['grip']=False
        await asyncio.sleep(.3)
        await scenario(state,wait,align)
    finally:
        writer.cancel();reader.cancel()
        await asyncio.gather(writer,reader,return_exceptions=True)
        await ws.close()


@pytest.mark.parametrize('side',['left','both'])
def test_episode_grippers_return_and_recording_exit_sleeps(tmp_path,side):
    with supervisor(tmp_path,side) as (proc,port,control):
        async def scenario(state,wait,align):
            active=await asyncio.to_thread(control)
            assert active['progress']['title']=='RECORDING'
            assert active['progress']['elapsed_s']>0
            await asyncio.to_thread(control,'record-save')
            finished=await wait(lambda s:s['state']=='HOLD' and 'grippers fully open' in s['reason']
                                and s['recording']['phase']=='finished')
            assert all(abs(f['q'][6]-1)<.01 for f in finished['feedback'].values())
            assert finished['progress']['title']=='EPISODE SAVED'
            elapsed=finished['progress']['elapsed_s']
            await asyncio.sleep(.1)
            assert (await asyncio.to_thread(control))['progress']['elapsed_s']==elapsed
            await asyncio.to_thread(control,'record-next')
            returned=await wait(lambda s:s['state']=='READY')
            assert all(abs(f['q'][6]-.8)<.01 for f in returned['feedback'].values())
            proc.send_signal(signal.SIGINT)
            await asyncio.to_thread(proc.wait,15)
        asyncio.run(exercise(port,control,scenario))
    log=[json.loads(line) for line in (tmp_path/'session/hardware.jsonl').read_text().splitlines()]
    assert any(row['event']=='sleep_torque_off' for row in log)
    assert any(row['event']=='sleep_start' and row['source']=='recording_exit' for row in log)


@pytest.mark.parametrize('side',['left','right'])
def test_single_arm_ignores_unselected_controller_loss_but_stops_for_selected(tmp_path,side):
    other='right' if side=='left' else 'left'
    with supervisor(tmp_path,side) as (_,port,control):
        async def scenario(state,wait,align):
            state['absent']=other
            await asyncio.sleep(.4)
            active=await asyncio.to_thread(control)
            assert active['state']=='ARMED',active
            assert active['recording']['phase']=='recording'
            assert active['progress']['title']=='RECORDING'
            state['absent']=side
            stopped=await wait(lambda s:s['state']=='HOLD')
            assert 'Controller tracking' in stopped['reason']
            await wait(lambda s:s['recording']['phase']=='paused')
        # Left-only must also start and align without ever seeing a right controller.
        asyncio.run(exercise(port,control,scenario,absent='right' if side=='left' else None))


@pytest.mark.parametrize('worker_expiry',[False,True])
def test_tracking_recovery_x_resumes_same_episode_and_timer(tmp_path,worker_expiry):
    reason='Controller target expired at motor execution' if worker_expiry else None
    with supervisor(tmp_path,'left',worker_hold_reason=reason) as (_,port,control):
        async def scenario(state,wait,align):
            first=await asyncio.to_thread(control)
            for _ in range(2):
                if worker_expiry:
                    (tmp_path/'inject-worker-hold').touch()
                else:
                    state['lost']=True
                paused=await wait(lambda s:s['state']=='HOLD' and s['recording']['phase']=='paused')
                elapsed=paused['progress']['elapsed_s']
                assert elapsed>0
                await asyncio.sleep(.5)
                assert (await asyncio.to_thread(control))['progress']['elapsed_s']==elapsed
                state['lost']=False
                await asyncio.sleep(.2)
                # Restored tracking alone does not resume capture.
                assert (await asyncio.to_thread(control))['recording']['phase']=='paused'
                state['grip']=True;state['x']=True
                await asyncio.sleep(.15)
                assert (await asyncio.to_thread(control))['recording']['phase']=='paused'
                state['grip']=False;state['x']=False
                await asyncio.sleep(.1)
                await align()
                resumed=await wait(lambda s:s['recording']['phase']=='recording')
                assert resumed['recording']['episode']==first['recording']['episode']
                assert elapsed <= resumed['progress']['elapsed_s'] < elapsed+1
            await asyncio.to_thread(control,'record-save')
            await wait(lambda s:s['recording']['saved']==1)
        asyncio.run(exercise(port,control,scenario,absent='right'))
    assert not list((tmp_path/'dataset/aborted').glob('*.hdf5'))
    accepted=list((tmp_path/'dataset/accepted').glob('*.hdf5'))
    assert len(accepted)==1
    import h5py
    import numpy as np
    from webxr_teleop.datasets.hdf5 import validate
    validate(accepted[0])
    with h5py.File(accepted[0],'r') as file:
        events=[(int(t),json.loads(v)) for t,v in zip(file['events/t_ns'][:],file['events/json'][:])]
        pauses=[t for t,v in events if v['kind']=='pause']
        resumes=[t for t,v in events if v['kind']=='resume']
        assert len(pauses)==len(resumes)==2
        def check(_,obj):
            if isinstance(obj,h5py.Group) and 't_ns' in obj:
                for begin,end in zip(pauses,resumes):
                    assert not np.any((obj['t_ns'][:]>=begin)&(obj['t_ns'][:]<end))
        file['streams'].visititems(check)
        from webxr_teleop.datasets.export_lerobot import alignment
        grid,_,_=alignment(file)
        for begin,end in zip(pauses,resumes):
            assert not np.any((grid>=begin)&(grid<end))


@pytest.mark.parametrize('reason',['Command watchdog expired','Target expired before serial write'])
def test_other_worker_holds_still_abort_recording(tmp_path,reason):
    with supervisor(tmp_path,'left',worker_hold_reason=reason) as (_,port,control):
        async def scenario(state,wait,align):
            (tmp_path/'inject-worker-hold').touch()
            status=await wait(lambda s:s['recording'].get('last_status')=='aborted')
            assert status['state']=='HOLD'
            assert reason in status['reason']
            assert status['recording']['saved']==0
        asyncio.run(exercise(port,control,scenario,absent='right'))
    assert len(list((tmp_path/'dataset/aborted').glob('*.hdf5')))==1
    assert not list((tmp_path/'dataset/accepted').glob('*.hdf5'))


def test_exit_during_recording_aborts_episode_and_sleeps(tmp_path):
    with supervisor(tmp_path,'left') as (proc,port,control):
        async def scenario(state,wait,align):
            proc.send_signal(signal.SIGINT)
            await asyncio.to_thread(proc.wait,15)
        asyncio.run(exercise(port,control,scenario))
    assert not list((tmp_path/'dataset/accepted').glob('*.hdf5'))
    assert len(list((tmp_path/'dataset/aborted').glob('*.hdf5')))==1
    log=[json.loads(line) for line in (tmp_path/'session/hardware.jsonl').read_text().splitlines()]
    assert any(row['event']=='sleep_torque_off' for row in log)


def test_terminal_group_interrupt_preserves_planner_for_sleep_and_finalizes_video(tmp_path):
    with supervisor(tmp_path,'left',process_group=True) as (proc,port,control):
        async def scenario(state,wait,align):
            os.killpg(proc.pid,signal.SIGINT)
            await asyncio.to_thread(proc.wait,15)
        asyncio.run(exercise(port,control,scenario,absent='right'))
    log=[json.loads(line) for line in (tmp_path/'session/hardware.jsonl').read_text().splitlines()]
    assert any(row['event']=='sleep_torque_off' for row in log)
    assert 'KeyboardInterrupt' not in (tmp_path/'supervisor.log').read_text()
    manifest=json.loads((tmp_path/'session/video/manifest.json').read_text())
    assert manifest['status']=='completed'


def test_stop_during_episode_gripper_release_stays_in_hold(tmp_path):
    with supervisor(tmp_path,'left') as (_,port,control):
        async def scenario(state,wait,align):
            await asyncio.to_thread(control,'record-save')
            await wait(lambda s:s['state']=='RELEASING')
            await asyncio.to_thread(control,'stop')
            stopped=await wait(lambda s:s['state']=='HOLD' and s['reason']=='Local stop command'
                and s['feedback']['left'].get('hold_command_confirmed')
                and s['feedback']['left']['reason']=='Local stop command')
            await asyncio.sleep(.3)
            later=await asyncio.to_thread(control)
            assert later['state']=='HOLD'
            assert later['feedback']['left']['q']==stopped['feedback']['left']['q']
        asyncio.run(exercise(port,control,scenario))


@pytest.mark.parametrize('side',['left','both'])
def test_two_episodes_hold_explicit_return_discard_and_resume(tmp_path,side):
    with supervisor(tmp_path,side) as (_,port,control):
        async def scenario(state,wait,align):
            before=await asyncio.to_thread(control)
            await asyncio.to_thread(control,'record-save','save-once')
            first=await wait(lambda s:s['recording']['phase']=='finished')
            assert first['recording']['saved']==1
            first=await wait(lambda s:s['state']=='HOLD' and 'grippers fully open' in s['reason'])
            assert all(f['q'][6]==pytest.approx(1.,abs=.01) for f in first['feedback'].values())
            held={s:f['q'] for s,f in first['feedback'].items()}
            await asyncio.sleep(.25)
            later=await asyncio.to_thread(control)
            assert {s:f['q'] for s,f in later['feedback'].items()}==held
            await asyncio.to_thread(control,'record-save','save-once')
            await asyncio.to_thread(control,'record-next','next-once')
            await wait(lambda s:s['state']=='READY')
            returned=await asyncio.to_thread(control)
            for s in returned['feedback']:
                assert returned['feedback'][s]['q'][6]==pytest.approx(.8,abs=.01)
            await align()
            # Exercise the Quest A edge; holding A must not save immediately.
            state['a']=True
            await wait(lambda s:s['recording']['phase']=='recording')
            await asyncio.sleep(.35)
            assert (await asyncio.to_thread(control))['recording']['phase']=='recording'
            state['a']=False;await asyncio.sleep(.05)
            await asyncio.to_thread(control,'record-discard','discard-once')
            final=await wait(lambda s:s['recording']['phase']=='finished')
            assert final['recording']['saved']==1
            assert final['recording']['last_status']=='discarded'
        asyncio.run(exercise(port,control,scenario))
    from webxr_teleop.datasets.hdf5 import validate
    accepted=list((tmp_path/'dataset/accepted').glob('*.hdf5'))
    assert len(accepted)==1
    validate(accepted[0])
    import h5py
    import numpy as np
    with h5py.File(accepted[0],'r') as file:
        assert np.max(np.ptp(file['streams/robot/left/observations/qpos'][:,:6],axis=0))>1e-5
    assert len(list((tmp_path/'dataset/discarded').glob('*.hdf5')))==1
    from webxr_teleop.datasets.export_lerobot import export
    result=export(tmp_path/'dataset',tmp_path/'lerobot','local/mock-acceptance')
    assert result['episodes'][0]['frames']>5


@pytest.mark.parametrize('failure',['stop','recorder'])
def test_recording_interruptions_abort_and_hold(tmp_path,failure):
    with supervisor(tmp_path) as (_,port,control):
        async def scenario(state,wait,align):
            if failure=='stop':
                # Local stop (also terminal Space) still aborts active capture.
                await asyncio.to_thread(control,'stop')
            else:
                status=await asyncio.to_thread(control)
                os.kill(status['processes']['recorder'],signal.SIGSTOP)
            stopped=await wait(lambda s:s['state']=='HOLD')
            assert stopped['recording']['saved']==0
            if failure=='recorder':
                os.kill(stopped['processes']['recorder'],signal.SIGCONT)
            await wait(lambda s:s['recording']['phase'] in ('finished','failed'))
        asyncio.run(exercise(port,control,scenario))
    assert not list((tmp_path/'dataset/accepted').glob('*.hdf5'))


def test_opening_timeout_allows_recording_after_alignment_at_current_pose(tmp_path):
    with supervisor(tmp_path,'left',slow_opening=True) as (_,port,control):
        async def scenario(state,wait,align):
            active=await asyncio.to_thread(control)
            assert active['progress']['title']=='READY TO RECORD'
            state['grip']=True
            await asyncio.sleep(.1)
            await asyncio.to_thread(control,'record-start')
            rejected=await wait(lambda s:'release selected side grips' in s['recording']['result'])
            assert rejected['recording']['phase']=='idle'
            state['grip']=False
            await asyncio.sleep(.4)
            await asyncio.to_thread(control,'record-start')
            await wait(lambda s:s['recording']['phase']=='recording')
            # Capture a real interval of robot and camera samples before save.
            await asyncio.sleep(.3)
            await asyncio.to_thread(control,'record-save')
            await wait(lambda s:s['recording']['saved']==1)
        asyncio.run(exercise(port,control,scenario,start=False))
    assert len(list((tmp_path/'dataset/accepted').glob('*.hdf5')))==1


def test_prepare_opening_before_first_episode(tmp_path):
    with supervisor(tmp_path,'left') as (_,port,control):
        async def scenario(state,wait,align):
            await asyncio.to_thread(control,'record-next')
            await wait(lambda s:s['state']=='READY' and 'Restart reached' in s['reason'])
            await align()
            await asyncio.to_thread(control,'record-start')
            status=await wait(lambda s:s['recording']['phase']=='recording')
            assert status['progress']['title']=='RECORDING'
            await asyncio.to_thread(control,'record-save')
            await wait(lambda s:s['recording']['saved']==1)
        asyncio.run(exercise(port,control,scenario,start=False))


def test_slow_planner_initialization_does_not_arm_worker_watchdog(tmp_path):
    with supervisor(tmp_path,'left',slow_planner=True) as (_,port,control):
        async def scenario(state,wait,align):
            assert (await asyncio.to_thread(control))['progress']['title']=='RECORDING'
            await asyncio.to_thread(control,'record-save')
            await wait(lambda s:s['recording']['saved']==1)
        asyncio.run(exercise(port,control,scenario,absent='right'))
    rows=[json.loads(line) for line in (tmp_path/'session/hardware.jsonl').read_text().splitlines()]
    assert not any('watchdog expired' in row.get('reason','').lower() for row in rows)


def test_two_full_resolution_cameras_capture_without_heartbeat_failure(tmp_path):
    with supervisor(tmp_path,'left',full_resolution=True) as (_,port,control):
        async def scenario(state,wait,align):
            for _ in range(30):
                await asyncio.sleep(.1)
                status=await asyncio.to_thread(control)
                assert status['recording']['error'] is None
                assert status['recording']['phase']=='recording'
            await asyncio.to_thread(control,'record-save')
            await wait(lambda s:s['recording']['saved']==1)
        asyncio.run(exercise(port,control,scenario,absent='right'))


@pytest.mark.parametrize('source',['quest','terminal'])
def test_discard_returns_to_opening_without_starting_capture(tmp_path,source):
    with supervisor(tmp_path,'left') as (_,port,control):
        async def scenario(state,wait,align):
            await asyncio.sleep(.35)
            state['grip']=False
            if source=='quest':
                state['b']=True
            else:
                await asyncio.to_thread(control,'record-discard')
            result=await wait(lambda s:s['recording']['phase']=='finished')
            assert result['recording']['last_status']=='discarded'
            assert result['recording']['saved']==0
            result=await wait(lambda s:s['state']=='READY')
            assert result['feedback']['left']['q'][6]==pytest.approx(.8,abs=.01)
            await asyncio.sleep(.25)
            assert (await asyncio.to_thread(control))['state']=='READY'
            state['b']=False
        asyncio.run(exercise(port,control,scenario))
    assert len(list((tmp_path/'dataset/discarded').glob('*.hdf5')))==1
    assert not list((tmp_path/'dataset/accepted').glob('*.hdf5'))


def test_gripper_fault_next_episode_recovers_and_can_record_again(tmp_path):
    with supervisor(tmp_path,'left',worker_hold_reason='Gripper encoder outside calibration',
                    worker_fault_code='gripper_range') as (_,port,control):
        async def scenario(state,wait,align):
            (tmp_path/'inject-worker-hold').touch()
            await wait(lambda s:s['state']=='FAULT' and s['recording'].get('last_status')=='aborted')
            await asyncio.sleep(.5)
            await asyncio.to_thread(control,'record-next')
            await wait(lambda s:s['state']=='READY',seconds=15)
            await align()
            await asyncio.to_thread(control,'record-start')
            await wait(lambda s:s['recording']['phase']=='recording')
            await asyncio.sleep(.3)
            await asyncio.to_thread(control,'record-save')
            await wait(lambda s:s['recording']['saved']==1)
        asyncio.run(exercise(port,control,scenario,absent='right'))
    assert len(list((tmp_path/'dataset/aborted').glob('*.hdf5')))==1
    assert len(list((tmp_path/'dataset/accepted').glob('*.hdf5')))==1


def test_quest_a_slow_record_start_keeps_workers_armed(tmp_path):
    with supervisor(tmp_path, slow_record_start=True) as (_,port,control):
        async def scenario(state,wait,align):
            state['a']=True
            await wait(lambda s:s['recording']['phase']=='starting')
            await asyncio.sleep(.2)
            status=await asyncio.to_thread(control)
            assert status['state']=='ARMED',status
            state['a']=False
            status=await wait(lambda s:s['recording']['phase']=='recording')
            assert status['state']=='ARMED',status
            await asyncio.sleep(.3)
            assert (await asyncio.to_thread(control))['state']=='ARMED'
        asyncio.run(exercise(port,control,scenario,start=False))
    rows=[json.loads(line) for line in (tmp_path/'session/hardware.jsonl').read_text().splitlines()]
    assert not any(row['event']=='hold' and 'expired' in row.get('reason','').lower() for row in rows)


def test_camera_discovery_longer_than_runtime_heartbeat_can_start(tmp_path):
    with supervisor(tmp_path,'left',slow_camera_discovery=True) as (_,port,control):
        async def scenario(state,wait,align):
            status=await asyncio.to_thread(control)
            assert status['state']=='ARMED',status
            assert status['recording']['phase']=='idle',status
            await asyncio.to_thread(control,'record-start')
            status=await wait(lambda s:s['recording']['phase']=='recording')
            assert status['state']=='ARMED',status
        asyncio.run(exercise(port,control,scenario,start=False))
