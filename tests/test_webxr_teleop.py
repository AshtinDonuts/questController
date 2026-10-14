import json
import asyncio
from functools import wraps
import math
from pathlib import Path
import ssl
import socket
import time
import urllib.request

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R
from websockets.asyncio.client import connect

from webxr_teleop.control import Teleop, Config
from webxr_teleop.run import MODEL
from webxr_teleop.server import ControllerState, VRServer, _parse_controller


def async_test(fn):
    @wraps(fn)
    def run(*args, **kwargs):
        return asyncio.run(fn(*args, **kwargs))
    return run


def controller(position=(0, 1.2, -0.4), grip=False, t=0, q=None, buttons=None, trigger=0):
    return ControllerState(np.array(position, dtype=float),
        R.identity().as_quat() if q is None else q,
        trigger, grip, buttons or {}, t)


@pytest.fixture
def arm():
    m = mujoco.MjModel.from_xml_path(str(MODEL))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return Teleop(m, d)


def align(arm):
    arm.step(controller(buttons={'x': True}), controller(), 0, 1/30)
    assert arm.calibrated
    assert np.allclose(arm.basis, [[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    arm.step(None, controller(grip=True, t=0.01), 0.01, 1/30)
    assert arm.engaged


@pytest.mark.parametrize('vr,world', [([1,0,0],[1,0,0]), ([0,0,-1],[0,1,0]), ([0,1,0],[0,0,1])])
def test_translation_axis_gain_and_speed(arm, vr, world):
    align(arm)
    origin, _ = arm.actual_pose()
    previous = arm.target_p.copy()
    for i in range(1, 151):
        t = i/30
        offset = np.array(vr)*0.1*min(i/60, 1)
        arm.step(None, controller(np.array([0,1.2,-0.4])+offset, grip=True, t=t), t, 1/30)
        assert np.linalg.norm(arm.target_p-previous) <= arm.cfg.linear_speed/30 + 1e-9
        previous = arm.target_p.copy()
        # Differential IK may briefly lag while honoring the joint-speed cap.
        assert arm.ik_solved
        assert arm.ik_position_error < .006
        assert arm.ik_joint_speed <= arm.cfg.joint_speed+1e-7
    assert arm.ik_ok
    actual, _ = arm.actual_pose()
    np.testing.assert_allclose(actual-origin, np.array(world)*0.1, atol=0.006)


@pytest.mark.parametrize('axis', [[1,0,0],[0,1,0],[0,0,1]])
def test_relative_rotation_and_return_to_identity(arm, axis):
    align(arm)
    _, origin = arm.actual_pose()
    q0 = R.identity()
    for i in range(1, 121):
        angle = math.radians(10)*min(i/30, 1)
        q = (R.from_rotvec(np.array(axis)*angle)*q0).as_quat()
        arm.step(None, controller(grip=True, t=i/30, q=q), i/30, 1/30)
    expected = R.from_rotvec(arm.basis@np.array(axis)*math.radians(10)).as_matrix()@origin
    _, actual = arm.actual_pose()
    assert np.linalg.norm(R.from_matrix(expected@actual.T).as_rotvec()) < math.radians(2)
    for i in range(121, 241):
        arm.step(None, controller(grip=True, t=i/30), i/30, 1/30)
    assert np.linalg.norm(R.from_matrix(origin@arm.actual_pose()[1].T).as_rotvec()) < math.radians(2)


def test_tracking_loss_and_jump_require_release(arm):
    align(arm)
    saved = arm.data.qpos.copy()
    arm.step(None, controller(grip=True, t=0), 1, 1/30)
    assert not arm.engaged
    arm.step(None, controller(grip=True, t=1.1), 1.1, 1/30)
    assert not arm.engaged
    np.testing.assert_allclose(arm.data.qpos, saved)
    arm.step(None, controller(t=1.2), 1.2, 1/30)
    arm.step(None, controller(grip=True, t=1.3), 1.3, 1/30)
    assert arm.engaged
    arm.step(None, controller([1,1.2,-.4], grip=True, t=1.4), 1.4, 1/30)
    assert not arm.engaged and 'tracking_jump' in arm.events
    arm.step(None, controller([1,1.2,-.4], grip=True, t=1.5), 1.5, 1/30)
    assert not arm.engaged


def test_clutch_repositions_without_jump(arm):
    align(arm)
    saved = arm.actual_pose()[0]
    arm.step(None, controller([1,1.2,-.4], t=1), 1, 1/30)
    arm.step(None, controller([1,1.2,-.4], grip=True, t=1.1), 1.1, 1/30)
    np.testing.assert_allclose(arm.target_p, saved, atol=1e-9)


def test_home_cancel_and_calibration_reset(arm):
    align(arm)
    arm.reset_tracking()
    assert not arm.engaged and not arm.calibrated
    arm.data.qpos[arm.qids[0]] += .2
    arm.start_home()
    previous = arm.data.qpos[arm.qids].copy()
    arm.step(None, controller(t=1), 1, 1/30)
    assert np.max(np.abs(arm.data.qpos[arm.qids]-previous)) <= arm.cfg.home_speed/30 + 1e-9
    arm.step(None, controller(t=1.1, buttons={'b': True}), 1.1, 1/30)
    assert not arm.homing


def test_invalid_ik_holds_joint_command_and_gripper_disconnect(arm):
    q = arm.data.qpos.copy()
    assert not arm.solve(np.array([np.nan, 0., 0.]), np.eye(3))
    np.testing.assert_array_equal(arm.data.qpos, q)
    arm.step(None, controller(trigger=1), 0, 1/30)
    closed = arm.data.qpos[arm.fingers].copy()
    arm.step(None, None, 1, 1/30)
    np.testing.assert_array_equal(arm.data.qpos[arm.fingers], closed)


def packet():
    return dict(tracked=True, position=dict(x=0, y=1, z=-.4),
                quaternion=dict(x=0,y=0,z=0,w=1), trigger=.5, buttons={'squeeze':True})


@pytest.mark.parametrize('change', [dict(tracked=False), dict(position={'x':float('nan'),'y':0,'z':0}),
    dict(quaternion={'x':0,'y':0,'z':0,'w':0}), dict(trigger='bad'), dict(buttons=3)])
def test_reject_invalid_tracking(change):
    assert _parse_controller({**packet(), **change}, 1) is None


def test_zero_position_valid_and_quaternion_normalized():
    p = packet();p['position'] = dict(x=0,y=0,z=0);p['quaternion']['w']=1.1
    state = _parse_controller(p, 1)
    assert state is not None and np.linalg.norm(state.quaternion) == pytest.approx(1)


@async_test
async def test_https_websocket_reset_disconnect_and_single_owner(tmp_path):
    from webxr_teleop import server as module
    old = module.DEFAULT_TLS_DIR
    module.DEFAULT_TLS_DIR = tmp_path / 'tls'
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    server = VRServer('127.0.0.1', port)
    module.DEFAULT_TLS_DIR = old
    server.start()
    ctx = ssl._create_unverified_context()
    try:
        with urllib.request.urlopen(server.url, context=ctx) as response:
            assert b'ViperX Simulator' in response.read()
        with urllib.request.urlopen(server.url+'/vendor/aframe.min.js', context=ctx) as response:
            assert len(response.read()) > 100000
        async with connect(f'wss://127.0.0.1:{port}', ssl=ctx) as ws:
            await ws.send(json.dumps({'type':'controllers','right':packet()}))
            deadline = time.monotonic()+2
            while server.latest()[1] is None and time.monotonic()<deadline:
                await asyncio.sleep(.01)
            assert server.latest()[1].squeeze
            gen=server.generation
            await ws.send(json.dumps({'type':'reset'}))
            deadline=time.monotonic()+2
            while server.generation==gen and time.monotonic()<deadline:
                await asyncio.sleep(.01)
            assert server.generation>gen and server.latest()[1] is None
            async with connect(f'wss://127.0.0.1:{port}', ssl=ctx) as second:
                with pytest.raises(Exception):
                    await asyncio.wait_for(second.recv(),1)
            # Invalid payload cannot terminate the controlling connection.
            await ws.send('[]')
            await ws.send(json.dumps({'type':'controllers','right':packet()}))
            server.publish({'type':'status','message':'test'})
            assert json.loads(await asyncio.wait_for(ws.recv(),2))['message']=='test'
        deadline=time.monotonic()+2
        while server.num_clients and time.monotonic()<deadline:
            await asyncio.sleep(.01)
        assert server.latest()[1] is None
    finally:
        server.stop()
    assert not server._thread.is_alive()


@pytest.mark.parametrize('scene', ['single', 'dual'])
@pytest.mark.parametrize('reset_during_recording', [False, True])
@async_test
async def test_live_runner_packets_episode_and_mp4(tmp_path, reset_during_recording, scene):
    """Exercise the actual entrypoint through TLS/WSS, not a direct mapper call."""
    import subprocess
    import sys
    from webxr_teleop.run import ROOT
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    output = tmp_path / 'session'
    log = tmp_path / 'runner.log'
    with log.open('w') as handle:
        process = subprocess.Popen([sys.executable, '-m', 'webxr_teleop.run', '--headless',
            '--scene', scene, '--host', '127.0.0.1', '--port', str(port), '--seconds', '7', '--output', str(output)],
            cwd=ROOT, stdout=handle, stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                if 'MP4:' in log.read_text():
                    break
                assert process.poll() is None, log.read_text()
                await asyncio.sleep(.05)
            else:
                raise AssertionError(log.read_text())
            async with connect(f'wss://127.0.0.1:{port}', ssl=ssl._create_unverified_context()) as ws:
                q = R.identity().as_quat()
                start=time.monotonic()
                while time.monotonic()-start<5.5:
                    t=time.monotonic()-start
                    right=packet();left=packet()
                    for ctrl in (right,left):
                        ctrl['quaternion']=dict(zip(('x','y','z','w'),q.tolist()))
                        ctrl['buttons']={}
                        ctrl['trigger']=0
                    left['buttons']['x']=.2<t<.4
                    right['buttons']['a']=(.6<t<.8 or 1.0<t<1.2 or 4.6<t<4.8)
                    right['buttons']['squeeze']=1.5<t<4.4
                    if scene == 'dual':
                        left['buttons']['squeeze'] = 1.5<t<4.4
                        left['position']['z'] = -.4-float(.07*np.clip((t-1.7)/2,0,1))
                    right['position']['x']=float(.07*np.clip((t-1.7)/2,0,1))
                    await ws.send(json.dumps({'type':'controllers','reference_epoch':int(reset_during_recording and t>3.5),'left':left,'right':right}))
                    await asyncio.sleep(1/60)
            assert process.wait(timeout=12)==0, log.read_text()
        finally:
            if process.poll() is None:
                process.terminate();process.wait(timeout=10)
    manifest=json.loads((output/'manifest.json').read_text())
    assert manifest['status']=='completed'
    assert manifest['episodes']
    if reset_during_recording:
        assert not any(e['saved'] for e in manifest['episodes'])
    else:
        assert any(e['saved'] for e in manifest['episodes'])
    rows=[json.loads(s) for s in (output/'telemetry.jsonl').read_text().splitlines()]
    assert any(r['engaged'] for r in rows)
    assert max(r['position'][0] for r in rows)-rows[0]['position'][0]>.05
    if scene == 'dual':
        assert max(r['arms']['left']['position'][1] for r in rows)-rows[0]['arms']['left']['position'][1]>.05
    assert manifest['ik_backend'] == 'mink'
    assert not rows[-1]['tracking'] and not rows[-1]['engaged']
    probe=subprocess.check_output(['ffprobe','-v','error','-show_entries','stream=nb_frames',
                                   '-of','json',str(output/'session.mp4')],text=True)
    assert int(json.loads(probe)['streams'][0]['nb_frames'])==len(rows)==manifest['frames']


def test_released_controller_clears_old_limit_warning(arm):
    align(arm)
    arm.clipped = True
    arm.ik_ok = False
    arm.step(None, controller(t=1), 1, 1/30)
    assert not arm.clipped and arm.ik_ok and not arm.engaged


def test_arbitrary_heading_calibration_preserves_gravity(arm):
    # The human needs to point forward once, not trace three perfectly perpendicular lines.
    q = R.from_euler('y', 37, degrees=True)
    assert arm.calibrate(q.as_quat())
    forward = q.apply([0, 0, -1])
    np.testing.assert_allclose(arm.basis@forward, [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(arm.basis@np.array([0, 1, 0]), [0, 0, 1], atol=1e-12)
    assert np.linalg.det(arm.basis) == pytest.approx(1)
    # Reject vertical pointing without corrupting the last valid alignment.
    before = arm.basis.copy()
    assert not arm.calibrate(R.from_euler('x', 90, degrees=True).as_quat())
    np.testing.assert_array_equal(arm.basis, before)


def test_home_reaches_neutral_and_requires_release(arm):
    align(arm)
    arm.data.qpos[arm.qids[0]] += .1
    mujoco.mj_forward(arm.model, arm.data)
    arm.start_home()
    reached = False
    for i in range(1, 61):
        arm.step(None, controller(t=i/30, grip=True), i/30, 1/30)
        reached |= 'home_reached' in arm.events
    assert reached and not arm.homing and not arm.engaged
    np.testing.assert_allclose(arm.data.qpos[arm.qids], arm.home, atol=1e-8)


@pytest.mark.parametrize('field,value', [
    ('position', {'x':[0], 'y':[1], 'z':[-.4]}),
    ('quaternion', {'x':[0], 'y':[0], 'z':[0], 'w':[1]}),
    ('trigger', 10**1000),
])
def test_reject_nested_or_overflowed_pose_fields(field,value):
    assert _parse_controller({**packet(),field:value},1) is None


@async_test
async def test_recenter_epoch_survives_missing_reset_message(tmp_path):
    from webxr_teleop import server as module
    old=module.DEFAULT_TLS_DIR
    module.DEFAULT_TLS_DIR=tmp_path/'tls'
    try:
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        server=VRServer('127.0.0.1',port)
    finally:
        module.DEFAULT_TLS_DIR=old
    server.start()
    try:
        async with connect(f'wss://127.0.0.1:{port}',ssl=ssl._create_unverified_context()) as ws:
            await ws.send(json.dumps({'type':'controllers','reference_epoch':1,'right':packet()}))
            deadline=time.monotonic()+2
            while server.snapshot()[1] is None and time.monotonic()<deadline:
                await asyncio.sleep(.01)
            _,right,gen=server.snapshot()
            assert right is not None
            # The standalone reset frame was dropped, but the next pose carries its epoch.
            new_packet=packet();new_packet['position']['x']=.25
            await ws.send(json.dumps({'type':'controllers','reference_epoch':2,'right':new_packet}))
            deadline=time.monotonic()+2
            while server.snapshot()[2]==gen and time.monotonic()<deadline:
                await asyncio.sleep(.01)
            _,right,new_gen=server.snapshot()
            assert new_gen>gen
            assert right is None or right.position[0]==.25  # never old pose with new generation
            # Same epoch is not another reset.
            await ws.send(json.dumps({'type':'controllers','reference_epoch':2,'right':new_packet}))
            await asyncio.sleep(.05)
            assert server.snapshot()[2]==new_gen
    finally:
        server.stop()


def test_eef_axes_are_site_frame_in_webxr(arm):
    # Both a turned operator heading and a rotated wrist must be accounted for.
    assert arm.calibrate(R.from_euler('y', 37, degrees=True).as_quat())
    arm.data.qpos[arm.qids[-1]] += 0.3
    mujoco.mj_forward(arm.model, arm.data)
    _, world_from_eef = arm.actual_pose()
    base_q, eef_q = arm.frames_in_vr()
    np.testing.assert_allclose(arm.basis @ R.from_quat(base_q).as_matrix(), np.eye(3), atol=1e-12)
    np.testing.assert_allclose(arm.basis @ R.from_quat(eef_q).as_matrix(), world_from_eef, atol=1e-12)
    # A displacement along any displayed EEF axis must resolve to that local axis.
    xr_to_eef = world_from_eef.T @ arm.basis
    for i in range(3):
        np.testing.assert_allclose(xr_to_eef @ R.from_quat(eef_q).as_matrix()[:, i],
                                   np.eye(3)[:, i], atol=1e-12)
