"""No Quest and no physical bus: exercise the wall-clock opening handoff."""
import json
import os
import signal
import socket
import subprocess
import sys
import time
import pytest
from webxr_teleop.hardware.run import DEFAULT_PROFILE


@pytest.mark.parametrize('interrupt',[False,True])
def test_opening_deadline_holds_current_pose_without_auto_teleop(tmp_path,interrupt):
    profile=json.loads(DEFAULT_PROFILE.read_text())
    profile['arms']['left']['opening_rad'][5]=1.5  # trajectory exceeds ten seconds
    source=tmp_path/'profile.json';source.write_text(json.dumps(profile))
    private=tmp_path/'private';private.mkdir(mode=0o700)
    endpoint=str(private/'control.sock');output=tmp_path/'session'
    with socket.socket() as listener:
        listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
    with (tmp_path/'output.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,'-m','webxr_teleop.hardware.run','--mode','mock',
            '--side','left','--sequence','--profile',str(source),'--socket',endpoint,
            '--port',str(port),'--output',str(output),'--seconds','18'],
            stdout=log,stderr=log,env=dict(os.environ,MUJOCO_GL='egl'))
        def control(action='status'):
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
                client.settimeout(2);client.connect(endpoint);client.sendall((action+'\n').encode())
                return json.loads(client.recv(65536))
        try:
            deadline=time.monotonic()+7
            while True:
                try:status=control()
                except (OSError,ValueError):status={}
                if status.get('state')=='OPENING':break
                assert time.monotonic()<deadline,status
                time.sleep(.05)
            start=time.monotonic()
            if interrupt:control('stop')
            while time.monotonic()-start<10.8:
                status=control()
                if not interrupt and status['state']=='READY':break
                assert status['state']==('HOLD' if interrupt else 'OPENING'),status
                time.sleep(.05)
            assert status['state']==('HOLD' if interrupt else 'READY'),status
            time.sleep(.15)
            held=control();assert held['feedback']['left']['state']=='HOLD'
            position=held['feedback']['left']['q']
            if not interrupt:
                assert abs(position[5]-1.5)>.03  # target deliberately not reached
                assert control('reset')['state']=='READY'  # headset still required for teleop
            time.sleep(.2)
            assert control()['feedback']['left']['q']==position
        finally:
            proc.send_signal(signal.SIGTERM)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
    rows=[json.loads(line) for line in (output/'hardware.jsonl').read_text().splitlines()]
    events=[r for r in rows if r['event']=='opening_time_limit']
    if interrupt:assert not events
    else:
        assert len(events)==1
        assert 10<=events[0]['duration']<10.2
