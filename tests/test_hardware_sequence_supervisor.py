"""Process-level acceptance on mock buses; uses a real local WebSocket connection."""
import asyncio
import json
import os
from pathlib import Path
import signal
import socket
import ssl
import subprocess
import sys
import time

import websockets
import pytest


@pytest.mark.parametrize('failure,side', [('tracking','left'),('stop','left'),('complete','left')])
def test_sequence_opening_clutch_sleep_and_interruptions(tmp_path,failure,side):
    listener=socket.socket(); listener.bind(('127.0.0.1',0)); port=listener.getsockname()[1]; listener.close()
    private=tmp_path/'private'; private.mkdir(mode=0o700)
    endpoint=str(private/'control.sock'); output=tmp_path/'session'
    from webxr_teleop.hardware.run import DEFAULT_PROFILE
    profile=json.loads(DEFAULT_PROFILE.read_text())
    profile['arms'][side]['initial_rad'][0]=-.04
    profile_path=tmp_path/'profile.json'; profile_path.write_text(json.dumps(profile))
    log=(tmp_path/'supervisor.log').open('w')
    proc=subprocess.Popen([sys.executable,'-m','webxr_teleop.hardware.run','--mode','mock','--sequence','--port',str(port),
        '--profile',str(profile_path),'--socket',endpoint,'--output',str(output),'--seconds','25']+['--side',side],env=dict(os.environ,MUJOCO_GL='egl'),stdout=log,stderr=log)
    def control(action='status'):
        client=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM); client.settimeout(2)
        try:
            client.connect(endpoint); client.sendall((action+'\n').encode())
            return json.loads(client.recv(65536))
        finally: client.close()
    async def exercise():
        ctx=ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT);ctx.check_hostname=False;ctx.verify_mode=ssl.CERT_NONE
        deadline=time.monotonic()+8
        ws=None
        while time.monotonic()<deadline:
            try:
                ws=await websockets.connect(f'wss://127.0.0.1:{port}',ssl=ctx); break
            except (OSError,TimeoutError): await asyncio.sleep(.05)
        assert ws is not None
        control_state=dict(x=False,y=False,lost=False,grip=False,dx=0.)
        token=[None]; running=[True]
        async def reader():
            async for raw in ws:
                message=json.loads(raw)
                if message['type']=='freshness':token[0]=message['token']
        async def stream():
            seq=0
            while running[0]:
                hand=dict(tracked=True,position=dict(x=control_state['dx'],y=1.2,z=-.4),quaternion=dict(x=0,y=0,z=0,w=1),trigger=0,
                    buttons=dict(squeeze=control_state['grip']))
                left=dict(hand,buttons=dict(squeeze=control_state['grip'],x=control_state['x'],y=control_state['y']))
                seq+=1
                await ws.send(json.dumps(dict(type='controllers',reference_epoch=1,freshness=token[0],sequence=seq,
                    left=None if control_state['lost'] else left,right=hand)))
                await asyncio.sleep(.015)
        reading=asyncio.create_task(reader()); streaming=asyncio.create_task(stream())
        try:
            await asyncio.sleep(4)
            status=await asyncio.to_thread(control); assert status['state']=='READY',status
            if failure=='complete':
                control_state.update(grip=True,x=True)
                await asyncio.sleep(.2)
                status=await asyncio.to_thread(control)
                assert status['state']=='READY' and 'release side grips' in status['reason'],status
                control_state.update(grip=False,x=False)
                await asyncio.sleep(.1)
            control_state['x']=True; await asyncio.sleep(.15);control_state['x']=False
            await asyncio.sleep(.2)
            status=await asyncio.to_thread(control); assert status['state']=='ARMED',status
            assert 'Aligned' in status['reason'],status
            control_state['grip']=True
            for i in range(40):
                control_state['dx']=i*.0004; await asyncio.sleep(.02)
            control_state['grip']=False
            await asyncio.sleep(.4)
            held=await asyncio.to_thread(control)
            await asyncio.sleep(.2)
            assert (await asyncio.to_thread(control))['feedback'][side]['q']==held['feedback'][side]['q']
            control_state['y']=True
            if failure=='complete':control_state['grip']=True
            await asyncio.sleep(1.15)
            status=await asyncio.to_thread(control); assert status['state']=='PARKING',status
            if failure in ('complete','tracking'):
                if failure=='tracking':control_state['lost']=True
                await asyncio.sleep(5)
                status=await asyncio.to_thread(control); assert status['state']=='PARKED',status
                assert status['feedback'][side]['health']['torque'] is False is False
                assert (await asyncio.to_thread(control,'reset'))['state']=='PARKED'
                return
            if failure=='stop':
                await asyncio.to_thread(control,'stop')
            elif failure=='planner':
                os.kill(status['processes']['planner'],signal.SIGSTOP)
                await asyncio.sleep(.25)
            elif failure=='motor':
                os.kill(status['processes']['motors']['right'],signal.SIGKILL)
                await asyncio.sleep(.2)
            elif failure=='renderer':
                os.kill(status['processes']['renderer'],signal.SIGKILL)
                await asyncio.sleep(.2)
                status=await asyncio.to_thread(control)
                assert status['state']=='PARKING'
                control_state['lost']=True
            else:
                control_state['lost']=True
            await asyncio.sleep(.12)
            status=await asyncio.to_thread(control); assert status['state']==('FAULT' if failure=='motor' else 'HOLD'),status
            if failure=='planner': assert 'watchdog' in status['reason'].lower()
            elif failure not in ('motor','stop'): assert 'tracking lost' in status['reason']
            await asyncio.sleep(.1)
            stopped=await asyncio.to_thread(control)
            if failure=='motor':
                assert stopped['feedback']['left']['hold_confirmed']
                assert not stopped['feedback']['right']['hold_confirmed']
                assert not stopped['feedback']['right']['available']
            else:
                assert all(f['state']=='HOLD' and f['hold_confirmed'] for f in stopped['feedback'].values())
            control_state.update(lost=False,grip=True,x=True,y=False,dx=.3)
            await asyncio.sleep(.3)
            status=await asyncio.to_thread(control)
            assert status['state']==('FAULT' if failure=='motor' else 'HOLD')
            assert {s:f['q'] for s,f in status['feedback'].items()}=={s:f['q'] for s,f in stopped['feedback'].items()}
            # Restored tracking or X while gripping must not resume motion.
            # A fresh X press with released grips explicitly re-arms and aligns.
            if failure in ('tracking','stop'):
                control_state.update(grip=False,x=False,dx=0.)
                await asyncio.sleep(.15)
                assert (await asyncio.to_thread(control))['state']=='HOLD'
                control_state['x']=True
                await asyncio.sleep(.2)
                status=await asyncio.to_thread(control)
                assert status['state']=='ARMED' and 'Aligned' in status['reason'],status

        finally:
            running[0]=False; streaming.cancel();reading.cancel()
            await asyncio.gather(streaming,reading,return_exceptions=True);await ws.close()
    try:
        asyncio.run(exercise())
    finally:
        proc.send_signal(signal.SIGTERM)
        try:proc.wait(timeout=7)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close()
    assert proc.returncode==0,(tmp_path/'supervisor.log').read_text()
