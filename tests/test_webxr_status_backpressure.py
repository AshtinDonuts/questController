"""UI backpressure must not queue statuses or block freshness processing."""
import asyncio
import json

from webxr_teleop.server import VRServer


class Socket:
    remote_address = ('mock', 0)

    def __init__(self):
        self.incoming = asyncio.Queue()
        self.sent = []
        self.release_status = asyncio.Event()
        self.block_status = False
        self.status_inflight = 0
        self.max_status_inflight = 0

    async def send(self, raw):
        msg = json.loads(raw)
        self.sent.append(msg)
        if msg['type'] == 'status':
            self.status_inflight += 1
            self.max_status_inflight = max(self.max_status_inflight, self.status_inflight)
            try:
                if self.block_status:
                    await self.release_status.wait()
            finally:
                self.status_inflight -= 1

    def __aiter__(self):
        return self

    async def __anext__(self):
        msg = await self.incoming.get()
        if msg is None:
            raise StopAsyncIteration
        return json.dumps(msg)


async def wait_for(predicate):
    async with asyncio.timeout(2):
        while not predicate():
            await asyncio.sleep(.005)


def test_latest_status_wins_while_controller_freshness_remains_independent(tmp_path):
    async def scenario():
        server = VRServer(cert=tmp_path/'unused.crt', key=tmp_path/'unused.key', hardware_freshness=True)
        server._loop = asyncio.get_running_loop()
        ws = Socket()
        ws.block_status = True
        server.publish(dict(type='status', value=0))
        task = asyncio.create_task(server._handle_ws(ws))
        try:
            await wait_for(lambda: ws.status_inflight == 1)
            for value in range(1, 1001):
                server.publish(dict(type='status', value=value))
            await wait_for(lambda: len([m for m in ws.sent if m['type']=='freshness']) >= 3)
            assert len([m for m in ws.sent if m['type']=='status']) == 1
            assert ws.max_status_inflight == 1
            token = [m['token'] for m in ws.sent if m['type']=='freshness'][-1]
            hand = dict(tracked=True, position=dict(x=0,y=1,z=0),
                        quaternion=dict(x=0,y=0,z=0,w=1), buttons={}, trigger=0)
            packet = dict(type='controllers', reference_epoch=1, sequence=1,
                          freshness=token, left=hand, right=None)
            await ws.incoming.put(packet)
            await wait_for(lambda: server.packet_count == 1)
            assert server.snapshot()[0] is not None
            ws.release_status.set()
            await wait_for(lambda: len([m for m in ws.sent if m['type']=='status']) == 2)
            assert [m['value'] for m in ws.sent if m['type']=='status'] == [0, 1000]
            # No watchdog relaxation: the same token is rejected once old.
            await asyncio.sleep(.16)
            await ws.incoming.put(dict(packet, sequence=2))
            await wait_for(lambda: server.snapshot()[0] is None)
        finally:
            await ws.incoming.put(None)
            await task
        assert ws.status_inflight == 0
    asyncio.run(scenario())
