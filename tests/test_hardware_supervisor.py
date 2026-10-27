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


@pytest.mark.parametrize('failure,side', [('tracking','both'),('planner','both'),('renderer','both'),('motor','both'),('tracking','left')])
def test_tracking_loss_during_home_latches_selected_arms_and_never_resumes(tmp_path,failure,side):
    listener=socket.socket(); listener.bind(('127.0.0.1',0)); port=listener.getsockname()[1]; listener.close()
    private=tmp_path/'private'; private.mkdir(mode=0o700)
    endpoint=str(private/'control.sock'); output=tmp_path/'session'
    log=(tmp_path/'supervisor.log').open('w')
    proc=subprocess.Popen([sys.executable,'-m','webxr_teleop.hardware.run','--mode','mock','--port',str(port),
        '--socket',endpoint,'--output',str(output),'--seconds','25']+(['--side',side,'--manual-start'] if side!='both' else []),env=dict(os.environ,MUJOCO_GL='egl'),stdout=log,stderr=log)
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
            await asyncio.sleep(6 if side!='both' else .5)
            status=await asyncio.to_thread(control,'reset'); assert status['state']=='ARMED',status
            control_state['x']=True; await asyncio.sleep(.15);control_state['x']=False
            await asyncio.sleep(.2)
            control_state['grip']=True
            for i in range(40):
                control_state['dx']=i*.0004; await asyncio.sleep(.02)
            control_state['grip']=False
            await asyncio.sleep(.4)
            control_state['y']=True
            await asyncio.sleep(1.15)
            status=await asyncio.to_thread(control); assert status['state']=='HOMING',status
            if failure=='planner':
                os.kill(status['processes']['planner'],signal.SIGSTOP)
                await asyncio.sleep(.25)
            elif failure=='motor':
                os.kill(status['processes']['motors']['right'],signal.SIGKILL)
                await asyncio.sleep(.2)
            elif failure=='renderer':
                os.kill(status['processes']['renderer'],signal.SIGKILL)
                await asyncio.sleep(.2)
                status=await asyncio.to_thread(control)
                assert status['state']=='HOMING'
                control_state['lost']=True
            else:
                control_state['lost']=True
            await asyncio.sleep(.12)
            status=await asyncio.to_thread(control); assert status['state']==('FAULT' if failure=='motor' else 'HOLD'),status
            if failure=='planner': assert any(x in status['reason'].lower() for x in ('watchdog','target expired'))
            elif failure!='motor': assert 'tracking lost' in status['reason']
            await asyncio.sleep(.1)
            stopped=await asyncio.to_thread(control)
            if failure=='motor':
                assert stopped['feedback']['left']['hold_confirmed']
                assert not stopped['feedback']['right']['hold_confirmed']
                assert not stopped['feedback']['right']['available']
            else:
                assert all(f['state']=='HOLD' and f['hold_confirmed'] for f in stopped['feedback'].values())
            control_state.update(lost=False,grip=True,y=False,dx=.3)
            await asyncio.sleep(.3)
            status=await asyncio.to_thread(control)
            assert status['state']==('FAULT' if failure=='motor' else 'HOLD')
            assert {s:f['q'] for s,f in status['feedback'].items()}=={s:f['q'] for s,f in stopped['feedback'].items()}
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
