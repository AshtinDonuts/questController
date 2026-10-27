"""Software hold commissioning tests; no physical motor access."""
import hashlib
import json
import sys
import time
from types import SimpleNamespace
import pytest
from webxr_teleop.hardware.bus import SoftwareHoldBus
from webxr_teleop.hardware.core import MOTORS, load_profile
from webxr_teleop.hardware.run import DEFAULT_PROFILE, spawn_worker
from webxr_teleop.hardware.ipc import receive, send


def test_software_mode_preserves_mapping_commissioning(tmp_path):
    p=json.loads(DEFAULT_PROFILE.read_text()); calibration=tmp_path/'cal.json'
    calibration.write_text('{}'); a=p['arms']['left']
    a.update(verified=True,verification_note='test',calibration=str(calibration),
             calibration_sha256=hashlib.sha256(b'{}').hexdigest())
    p['watchdog_verified']=False; path=tmp_path/'rig.json'; path.write_text(json.dumps(p))
    with pytest.raises(ValueError,match='bus-watchdog'): load_profile(path,live=True,sides=('left',))
    load_profile(path,live=True,sides=('left',),require_motor_watchdog=False)
    a['verified']=False;path.write_text(json.dumps(p))
    with pytest.raises(ValueError,match='commissioning'):load_profile(path,live=True,sides=('left',),require_motor_watchdog=False)


@pytest.mark.parametrize('watchdog,torque,accepted',[(0,0,True),(15,0,True),(15,1,False),(255,0,False)])
def test_software_policy_never_clears_latch_or_changes_powered_configuration(watchdog,torque,accepted):
    b=SoftwareHoldBus.__new__(SoftwareHoldBus); regs={'Bus_Watchdog':[watchdog]*9,'Torque_Enable':[torque]*9};writes=[]
    b.register=lambda name:regs[name][:]; b.dxl=SimpleNamespace(COMM_SUCCESS=0)
    def write(port,idx,address,value):
        writes.append((idx,address,value));regs['Bus_Watchdog'][idx]=value;return 0,0
    b.bus=SimpleNamespace(motors={n:(i,) for i,n in enumerate(MOTORS)},port_handler=None,
                          packet_handler=SimpleNamespace(write1ByteTxRx=write))
    if accepted:
        b.watchdog(); assert regs['Bus_Watchdog']==[0]*9
        assert len(writes)==(9 if watchdog else 0)
        assert all(address==98 and value==0 for _,address,value in writes)
    else:
        with pytest.raises(RuntimeError):b.watchdog()
        assert not writes
    assert regs['Torque_Enable']==[torque]*9


@pytest.mark.parametrize('cancel',[False,True])
def test_manual_countdown_delays_torque_and_can_cancel(cancel):
    start=time.monotonic();sock,proc=spawn_worker(DEFAULT_PROFILE,'left','mock',sys.executable,startup=True,manual_start=True)
    messages=[]
    try:
        while time.monotonic()-start<1:
            messages+=receive(sock);time.sleep(.02)
        assert not any(m['type'] in ('startup','feedback') for m in messages)
        if cancel:send(sock,dict(op='shutdown'))
        deadline=start+8
        while time.monotonic()<deadline:
            messages+=receive(sock)
            if any(m['type']=='feedback' for m in messages) or proc.poll() is not None:break
            time.sleep(.02)
        if cancel:
            assert not any(m['type']=='startup' for m in messages)
            assert proc.wait(timeout=2)==0
        else:
            assert time.monotonic()-start>=5
            assert any(m['type']=='startup' and m['details']['verified'] for m in messages)
            assert next(m for m in messages if m['type']=='feedback')['state']=='HOLD'
    finally:
        if proc.poll() is None:send(sock,dict(op='shutdown'));proc.wait(timeout=2)
        sock.close()


def test_hold_drift_fault_keeps_original_target_and_rejects_reset():
    import os
    import subprocess
    from webxr_teleop.hardware.ipc import pair
    parent,child=pair()
    script='''
from webxr_teleop.hardware import worker
class DriftingBus(worker.MockBus):
    def read(self, **kwargs):
        q,health=super().read(**kwargs)
        if self.writes:q[0]+=.04
        return q,health
worker.MockBus=DriftingBus
worker.main()
'''
    proc=subprocess.Popen([sys.executable,'-c',script,'--profile',str(DEFAULT_PROFILE),
        '--side','left','--mode','mock','--fd',str(child.fileno()),'--parent-pid',str(os.getpid())],pass_fds=[child.fileno()])
    child.close();messages=[]
    try:
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            messages+=receive(parent)
            if any(m['type']=='fault' for m in messages):break
            time.sleep(.02)
        fault=next(m for m in messages if m['type']=='fault')
        assert 'Hold drift' in fault['reason'] and not fault['hold_confirmed']
        original=next(m for m in messages if m['type']=='feedback')['goal']
        send(parent,dict(op='arm',epoch=10,issued=time.monotonic()))
        time.sleep(.1)
        feedback=[m for m in receive(parent) if m['type']=='feedback']
        assert feedback and all(m['state']=='HOLD' and m['goal']==original for m in feedback)
        assert all(not m['hold_confirmed'] for m in feedback)
    finally:
        if proc.poll() is None:send(parent,dict(op='shutdown'));proc.wait(timeout=2)
        parent.close()
