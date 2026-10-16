"""Sleep recovery through the local socket, with mock motors and no headset."""
import json
import os
import signal
import socket
import subprocess
import sys
import time
import pytest
from webxr_teleop.hardware.run import DEFAULT_PROFILE


@pytest.mark.parametrize('interrupt_opening',[False,True])
def test_local_sleep_from_hold_without_tracking_and_stop(tmp_path,interrupt_opening):
    profile=json.loads(DEFAULT_PROFILE.read_text())
    arm=profile['arms']['left']
    arm['opening_rad']=arm['initial_rad'][:]
    if interrupt_opening:arm['opening_rad'][5]+=.15
    arm['sleep_rad']=arm['initial_rad'][:]
    arm['sleep_rad'][5]+=.08
    source=tmp_path/'profile.json';source.write_text(json.dumps(profile))
    private=tmp_path/'private';private.mkdir(mode=0o700)
    endpoint=str(private/'control.sock')
    with socket.socket() as listener:
        listener.bind(('127.0.0.1',0));port=listener.getsockname()[1]
    with (tmp_path/'output.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,'-m','webxr_teleop.hardware.run','--mode','mock',
            '--side','left','--sequence','--profile',str(source),'--socket',endpoint,
            '--port',str(port),'--output',str(tmp_path/'session'),'--seconds','20'],
            stdout=log,stderr=log,env=dict(os.environ,MUJOCO_GL='egl'))
        def control(action='status'):
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
                client.settimeout(2);client.connect(endpoint);client.sendall((action+'\n').encode())
                return json.loads(client.recv(65536))
        def wait_state(expected, seconds=8):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:
                try:status=control()
                except (OSError,ValueError):status={}
                if status.get('state')==expected:return status
                time.sleep(.04)
            raise AssertionError((expected,status,(tmp_path/'output.log').read_text()))
        try:
            wait_state('OPENING' if interrupt_opening else 'READY')
            if interrupt_opening:time.sleep(.5)
            else:assert control('stop')['state']=='HOLD'
            time.sleep(.04)  # no 300 ms settling requirement
            assert control('sleep')['state']=='PARKING'
            time.sleep(.5)
            assert control()['state']=='PARKING'  # no tracking still permits local sleep
            assert control('stop')['state']=='HOLD'
            assert control()['feedback']['left']['health']['torque'] is True
            time.sleep(.5)
            assert control()['state']=='HOLD'  # no automatic resume
            assert control('sleep')['state']=='PARKING'
            parked=wait_state('PARKED')
            assert parked['feedback']['left']['health']['torque'] is False
            time.sleep(.2)
            assert control()['feedback']['left']['health']['torque'] is False
            assert control('sleep')['state']=='PARKED'
        finally:
            proc.send_signal(signal.SIGTERM)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
