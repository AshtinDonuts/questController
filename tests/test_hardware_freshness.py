import asyncio
import json
import queue
import ssl
import time
import numpy as np
from webxr_teleop.server import VRServer


def test_hardware_websocket_rejects_replay_and_expired_tokens(tmp_path):
    """Exercise the actual websocket handler without a network or certificate setup."""
    server=object.__new__(VRServer)
    import threading
    server._lock=threading.Lock(); server._clients=set(); server.generation=0
    server._controllers={'left':None,'right':None}; server._status={}
    server.hardware_freshness=True; server.packet_count=0
    class Peer:
        remote_address=('test',0)
        def __init__(self): self.inbox=asyncio.Queue(); self.outbox=asyncio.Queue()
        async def send(self,message): await self.outbox.put(json.loads(message))
        def __aiter__(self): return self
        async def __anext__(self):
            m=await self.inbox.get()
            if m is None: raise StopAsyncIteration
            return json.dumps(m)
    async def exercise():
        peer=Peer(); task=asyncio.create_task(server._handle_ws(peer))
        token=(await peer.outbox.get())['token']
        hand=dict(tracked=True,position=dict(x=0,y=1,z=0),quaternion=dict(x=0,y=0,z=0,w=1),trigger=0,buttons={})
        msg=dict(type='controllers',reference_epoch=1,sequence=1,freshness=token,left=hand,right=hand)
        await peer.inbox.put(msg); await asyncio.sleep(.01)
        assert server.snapshot()[0] is not None
        await peer.inbox.put(msg); await asyncio.sleep(.01)
        assert server.snapshot()[0] is None  # duplicated sequence
        await asyncio.sleep(.16)
        msg['sequence']=2; await peer.inbox.put(msg); await asyncio.sleep(.01)
        assert server.snapshot()[0] is None  # old token cannot rejuvenate buffered poses
        await peer.inbox.put(None); await task
        assert server.snapshot()[:2]==(None,None)
    asyncio.run(exercise())
