"""Endpoint excursions are observations, never widened command limits."""
from types import SimpleNamespace
import numpy as np
import pytest
from webxr_teleop.hardware.gripper import feedback_aperture
from webxr_teleop.hardware.bus import LeRobotBus
from webxr_teleop.hardware.core import JOINTS

CAL=dict(closed_raw=2472,open_raw=3099)


@pytest.mark.parametrize('raw,expected,state',[(2472,0.,'in_range'),(3099,1.,'in_range'),
    (2471,0.,'endpoint_tolerance'),(2470,0.,'endpoint_tolerance'),(3100,1.,'endpoint_tolerance'),
    (3101,1.,'endpoint_tolerance'),(2467,0.,'out_of_range'),(3104,1.,'out_of_range')])
def test_feedback_status_and_projection(raw,expected,state):
    aperture,info=feedback_aperture(raw,CAL)
    assert aperture==expected and info['state']==state
    assert info['raw_count']==raw
    assert info['unclamped_aperture']==pytest.approx((raw-2472)/627)


@pytest.mark.parametrize('raw',[2400,3200,-1,65535])
def test_implausible_encoder_excursions_remain_faults(raw):
    with pytest.raises(ValueError,match=f'raw={raw}'):feedback_aperture(raw,CAL)


def test_descending_calibration_endpoint_tolerance():
    assert feedback_aperture(3100,dict(closed_raw=3099,open_raw=2472))[0]==0.
    assert feedback_aperture(2471,dict(closed_raw=3099,open_raw=2472))[0]==1.


@pytest.mark.parametrize('raw',[2471,3100,2467,3104])
def test_real_adapter_decode_and_exact_hold_never_emit_out_of_range_goal(raw):
    bus=LeRobotBus.__new__(LeRobotBus);bus.np=np;bus.gripper_calibration=CAL
    bus.cfg=dict(sign=[1]*6,offset_deg=[0]*6);bus.command_names=list(JOINTS)+['gripper']
    bus.bus=SimpleNamespace(apply_calibration=lambda values,names:np.array([0.]*8+[(values[-1]-2472)*100/627]))
    counts=np.array([0]*8+[raw],dtype=np.int32)
    q,_=bus.decode(counts)
    bus.raw=counts;bus.last_q=q[:]
    goal=bus.raw_target(q)
    assert 2472<=goal[-1]<=3099
    assert bus.gripper_feedback['raw_count']==raw
    assert (bus.gripper_feedback['unclamped_aperture']<0 if raw<2472 else bus.gripper_feedback['unclamped_aperture']>1)


def test_startup_rejects_larger_gripper_excursion_before_enable_or_goals():
    from test_hardware_startup import FakeFollower
    bus=FakeFollower()
    original_decode=bus.decode
    def outside(raw):
        q,v=original_decode(raw);bus.gripper_feedback={'state':'out_of_range'};return q,v
    bus.decode=outside
    with pytest.raises(ValueError,match='Startup rejected: gripper'):bus.startup_hold()
    assert not any(event[0] in ('torque','goals','caps') for event in bus.events)


@pytest.mark.parametrize('raw,expected',[(2471,2472),(3100,3099)])
def test_startup_raw_hold_bounds_gripper_goal(raw,expected):
    from webxr_teleop.hardware.core import MOTORS
    bus=LeRobotBus.__new__(LeRobotBus);bus.np=np;bus.gripper_calibration=CAL;goals={}
    class Writer:
        def addParam(self,idx,data):goals[idx]=int.from_bytes(bytes(data),'little')
        def txPacket(self):return 0
    bus.bus=SimpleNamespace(motors={n:(i,) for i,n in enumerate(MOTORS)},port_handler=None,packet_handler=None)
    bus.dxl=SimpleNamespace(COMM_SUCCESS=0,GroupSyncWrite=lambda *args:Writer())
    bus.register=lambda name:[goals[i] for i in range(9)]
    bus.write_raw_hold(np.array([0]*8+[raw],dtype=np.int32))
    assert goals[8]==expected


@pytest.mark.parametrize('raw',[2467,0,65535])
def test_startup_raw_hold_refuses_large_projection_before_any_packet(raw):
    bus=LeRobotBus.__new__(LeRobotBus);bus.np=np;bus.gripper_calibration=CAL;bus.bus=SimpleNamespace()
    with pytest.raises(ValueError):bus.write_raw_hold(np.array([0]*8+[raw],dtype=np.int32))
