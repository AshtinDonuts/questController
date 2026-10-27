"""Explicit restart tests use only encoder fixtures and mock subprocesses."""
import copy
import math
import numpy as np
import pytest
from webxr_teleop.hardware.recovery import RestartWindow
from webxr_teleop.hardware.planner import Planner
from test_hardware_planner import setup


def test_restart_readiness_ignores_old_hold_error_but_requires_encoder_stability():
    window=RestartWindow();q=[.1]*6+[.5]
    for i in range(16):ready=window.update(q,i*.02)
    assert ready  # Any old hold goal is irrelevant to readiness.
    window=RestartWindow()
    for i in range(100):assert not window.update(q,0.)  # Cached data cannot establish stability.
    for i in range(1,30):assert not window.update([.1+i*.001]*6+[.5],i*.02)
    for i in range(30,47):ready=window.update([.129]*6+[.5],i*.02)
    assert ready
    assert not window.update(q,2.)  # Feedback gap resets readiness.


def test_restart_path_check_precedes_motion(monkeypatch):
    p,q,h=setup();planner=Planner(p,q)
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:0.)
    monkeypatch.setattr(planner,'valid_path',lambda *args,**kwargs:False)
    with pytest.raises(ValueError,match='Restart path'):
        planner.step(dict(t=0.,epoch=1,seq=1,mode='hold',prepare_restart=True,restart=True,measured=q,hands=h))
    assert {s:x.tolist() for s,x in planner.last.items()}==q


def test_restart_uses_joint_trajectory_without_ik_and_preserves_gripper(monkeypatch):
    p,q,h=setup();original=copy.deepcopy(q)
    for s in q:p['arms'][s]['opening_rad']=q[s][:6];p['arms'][s]['opening_rad'][5]+=.2
    planner=Planner(p,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    for ik in planner.iks.values():
        monkeypatch.setattr(ik,'advance',lambda *args:pytest.fail('Restart must not solve Cartesian IK'))
    poses=[]
    for i in range(250):
        clock[0]=i*.02
        r=planner.step(dict(t=clock[0],epoch=1,seq=i,mode='opening',restart=True,home_id=1,measured=q,hands={'left':None,'right':None}))
        q=r['q'];poses.append(q['left'][:6])
        assert q['left'][6]==original['left'][6]
    assert r['home_done']
    np.testing.assert_allclose(q['left'][:6],p['arms']['left']['opening_rad'])
    v=np.diff(poses,axis=0)/.02
    assert np.max(abs(v))<=math.radians(10)+1e-8
    assert np.max(abs(np.diff(v,axis=0)/.02))<=math.radians(30)+1e-8


@pytest.mark.parametrize('side', ['left', 'right', 'both'])
@pytest.mark.parametrize('error_deg', [1.58, 4.9, 5.0])
def test_restart_settles_with_reported_residual_error(monkeypatch, side, error_deg):
    p,q,_=setup()
    sides=('left','right') if side=='both' else (side,)
    for s in sides:
        q[s]=p['arms'][s]['opening_rad'][:]+[.8]
        q[s][0]+=math.radians(error_deg)
    planner=Planner(p,q,sides);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    completed=False
    for i in range(230):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],epoch=1,seq=i,mode='opening',restart=True,
            home_id=1,measured=q,hands={}))
        if clock[0]<planner.home.duration+.3-1e-9:
            assert not result['home_done']
        completed|=result['home_done']
    assert completed
    assert result['arrival_error_deg']==pytest.approx(error_deg)


@pytest.mark.parametrize('error_deg,moving', [(5.1,False),(1.58,True)])
def test_restart_still_faults_when_outside_tolerance_or_not_stationary(monkeypatch,error_deg,moving):
    p,q,_=setup()
    for s in q:
        q[s]=p['arms'][s]['opening_rad'][:]+[.8]
        q[s][0]+=math.radians(error_deg)
    planner=Planner(p,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    with pytest.raises(ValueError,match='Opening settling timed out'):
        for i in range(300):
            clock[0]=i*.02
            if moving:
                # Both poses are inside 5 degrees, but their excursion exceeds
                # the two-encoder-count stationarity bound.
                for s in q:q[s][0]=p['arms'][s]['opening_rad'][0]+math.radians(1.58 if i%2 else 1.0)
            result=planner.step(dict(t=clock[0],epoch=1,seq=i,mode='opening',restart=True,
                home_id=1,measured=q,hands={}))
            assert not result['home_done']


RUNNER='''
import json,sys,os,subprocess
from pathlib import Path
from webxr_teleop.hardware import run,planner,worker
from webxr_teleop.hardware.ipc import pair
Original=planner.Planner
class FaultPlanner(Original):
    def step(self,job):
        if job['mode']=='opening' and not job.get('restart'):
            kind=self.profile.get('injected_fault')
            if kind=='ik':raise ValueError('IK rejected target: QP solver daqp failed to find a solution.')
            if kind=='geometry':raise ValueError('Finite target path violates joint/planar limits')
        return super().step(job)
planner.Planner=FaultPlanner
if __name__=='__main__':
    if '--fake-worker' in sys.argv:
        sys.argv.remove('--fake-worker')
        marker=Path(sys.argv.pop(1))
        class DriftingBus(worker.MockBus):
            def read(self,**kwargs):
                q,h=super().read(**kwargs)
                if marker.exists():
                    if marker.read_text().strip()=='gripper':
                        q[6]=0.;h['gripper_feedback']=dict(state='out_of_range',outside_counts=5,raw_count=2467,unclamped_aperture=-5/627)
                    else:q[0]+=.04
                return q,h
        worker.MockBus=DriftingBus
        worker.main()
    else:
        original_spawn=run.spawn_worker
        def spawn(profile,side,mode,python,**kwargs):
            parent,child=pair()
            marker=Path(profile).parent/'drift'
            args=[python,__file__,str(marker),'--fake-worker','--profile',str(profile),'--side',side,
                  '--mode','mock','--fd',str(child.fileno()),'--parent-pid',str(os.getpid()),'--startup','--sequence']
            proc=subprocess.Popen(args,pass_fds=[child.fileno()]);child.close()
            return parent,proc
        run.spawn_worker=spawn
        run.main()
'''


@pytest.mark.parametrize('kind,side',[('ik','left'),('drift','left'),('gripper','left'),('geometry','left'),('ik','both')])
def test_keyboard_restart_from_latched_fault_without_headset(tmp_path,kind,side):
    import json,os,pty,signal,socket,subprocess,sys,time
    from webxr_teleop.hardware.run import DEFAULT_PROFILE,ROOT
    p=json.loads(DEFAULT_PROFILE.read_text());p['injected_fault']=kind
    sides=('left','right') if side=='both' else (side,)
    for s in sides:
        p['arms'][s]['opening_rad']=p['arms'][s]['initial_rad'][:]
        # Recovery takes >10 s in IK case, proving initial-opening timeout is not used.
        if kind=='ik':p['arms'][s]['opening_rad'][5]+=1.
    source=tmp_path/'profile.json';source.write_text(json.dumps(p))
    runner=tmp_path/'runner.py';runner.write_text(RUNNER)
    private=tmp_path/'private';private.mkdir(mode=0o700);endpoint=str(private/'control.sock')
    with socket.socket() as listener:listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
    master,slave=pty.openpty()
    with (tmp_path/'output.log').open('w') as log:
        env=dict(os.environ,MUJOCO_GL='egl',PYTHONPATH=str(ROOT))
        proc=subprocess.Popen([sys.executable,str(runner),'--mode','mock','--side',side,'--sequence',
            '--profile',str(source),'--socket',endpoint,'--port',str(port),'--output',str(tmp_path/'session')],
            stdin=slave,stdout=log,stderr=log,env=env)
        os.close(slave)
        def control(action='status'):
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as c:
                c.settimeout(2);c.connect(endpoint);c.sendall((action+'\n').encode());return json.loads(c.recv(65536))
        def wait(expected,seconds=8):
            deadline=time.monotonic()+seconds;status={}
            while time.monotonic()<deadline:
                try:status=control()
                except (OSError,ValueError):pass
                if status.get('state')==expected:return status
                time.sleep(.03)
            raise AssertionError((expected,status,(tmp_path/'output.log').read_text()))
        try:
            if kind in ('drift','gripper'):
                wait('READY');(tmp_path/'drift').write_text('gripper' if kind=='gripper' else '')
            before=wait('FAULT')
            if kind=='drift':
                assert 'Hold drift' in before['reason'];(tmp_path/'drift').unlink()
            if kind=='gripper':
                assert 'Gripper encoder outside calibration' in before['reason']
                os.write(master,b'k');time.sleep(.1)
                rejected=control()
                assert rejected['state']=='FAULT' and 'gripper encoder still outside' in rejected['reason']
                (tmp_path/'drift').unlink()
            if kind=='geometry':
                os.write(master,b'k');time.sleep(.1)
                status=control();assert status['state']=='FAULT' and 'nonrecoverable' in status['reason']
                return
            time.sleep(.4)
            recovery_apertures={s:f['q'][6] for s,f in control()['feedback'].items()}
            os.write(master,b'k')  # Actual keyboard shortcut, no Enter.
            moving=wait('OPENING')
            assert moving['processes']['planner']!=before['processes']['planner']
            assert moving['processes']['motors']==before['processes']['motors']
            assert 'already in progress' in control('restart')['reason']
            ready=wait('READY',seconds=16)
            assert 'Restart reached starting pose' in ready['reason']
            for s in sides:
                assert ready['feedback'][s]['health']['torque'] is True
                assert ready['feedback'][s]['q'][6]==pytest.approx(recovery_apertures[s])
                np.testing.assert_allclose(ready['feedback'][s]['q'][:6],p['arms'][s]['opening_rad'],atol=math.radians(.5))
            assert control('stop')['state']=='HOLD'
            time.sleep(.1);assert control()['state']=='HOLD'
        finally:
            proc.send_signal(signal.SIGTERM)
            try:proc.wait(timeout=6)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
            os.close(master)


def test_restart_control_client_is_not_a_hardware_owner():
    from webxr_teleop.hardware.endpoint import is_hardware_process
    assert not is_hardware_process(['python','-m','webxr_teleop.hardware.run','restart'])


def test_stops_cancel_restart_without_automatic_resume(tmp_path):
    import os,sys,time
    from webxr_teleop.hardware.run import spawn_worker,DEFAULT_PROFILE
    from webxr_teleop.hardware.ipc import send,receive
    sock,proc=spawn_worker(DEFAULT_PROFILE,'left','mock',sys.executable,startup=True,sequence=True)
    def feedback(predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            for m in receive(sock):
                assert m.get('type')!='fault',m
                if m.get('type')=='feedback' and predicate(m):return m
            time.sleep(.005)
        raise AssertionError('Missing feedback')
    try:
        q=feedback(lambda m:True)['q']
        send(sock,dict(op='restart',epoch=1,issued=time.monotonic()))
        feedback(lambda m:m['state']=='ARMED' and m['epoch']==1)
        now=time.monotonic();target=q[:];target[5]+=.0001
        send(sock,dict(op='command',seq=1,epoch=1,issued=now,q=target,mode='opening',freeze=False))
        feedback(lambda m:m['goal']==target)
        send(sock,dict(op='hold',reason='Stop during restart'))
        stopped=feedback(lambda m:m['state']=='HOLD' and m['reason']=='Stop during restart')
        now=time.monotonic();target[5]+=.001
        send(sock,dict(op='command',seq=2,epoch=1,issued=now,q=target,mode='opening',freeze=False))
        f=feedback(lambda m:m['t']>now)
        assert f['state']=='HOLD' and f['goal']==stopped['goal']
    finally:
        if proc.poll() is None:send(sock,dict(op='shutdown'));proc.wait(timeout=3)
        sock.close()
