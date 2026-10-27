from types import SimpleNamespace
import numpy as np
import pytest
from webxr_teleop.hardware.bus import LeRobotBus
from webxr_teleop.hardware.core import MOTORS


def fake(tuning):
    bus=LeRobotBus.__new__(LeRobotBus)
    bus.cfg={} if tuning is None else {'motor_tracking_profile':tuning}
    values={'Profile_Velocity':[3]*9,'Profile_Acceleration':[1]*9}
    bus.preflight={**{k:v[:] for k,v in values.items()},'Drive_Mode':[0]*9}
    bus.register=lambda key:values[key][:]
    def write(port,idx,address,value):
        values[{112:'Profile_Velocity',108:'Profile_Acceleration'}[address]][idx]=value
        return 0,0
    bus.dxl=SimpleNamespace(COMM_SUCCESS=0)
    bus.bus=SimpleNamespace(calibration={'motor_names':list(MOTORS),'start_pos':[0]*9,'end_pos':[1000]*9},
        motors={n:(i,) for i,n in enumerate(MOTORS)},port_handler=None,
        packet_handler=SimpleNamespace(write4ByteTxRx=write))
    return bus,values


def test_tuned_arm_profiles_and_gripper_preservation():
    bus,v=fake({'velocity_counts':14,'acceleration_counts':4})
    bus.configure_motion_caps()
    assert v['Profile_Velocity']==[14]*8+[18]
    assert v['Profile_Acceleration']==[4]*8+[5]
    bus.opening_speed(True);assert v['Profile_Velocity']==[24]*8+[18]
    bus.opening_speed(False);assert v['Profile_Velocity']==[14]*8+[18]


def test_untuned_caps_remain_conservative():
    bus,v=fake(None);bus.configure_motion_caps()
    assert v['Profile_Velocity']==[3]*8+[18]
    assert v['Profile_Acceleration']==[1]*8+[5]


@pytest.mark.parametrize('value',[0,-1,True,1.5,32768])
def test_invalid_profile_refused_before_writes(value):
    bus,v=fake({'velocity_counts':value,'acceleration_counts':4})
    with pytest.raises(ValueError):bus.configure_motion_caps()
    assert v['Profile_Velocity']==[3]*9


def test_time_based_mode_refused():
    bus,v=fake({'velocity_counts':14,'acceleration_counts':4});bus.preflight['Drive_Mode'][0]=4
    with pytest.raises(ValueError,match='velocity-based'):bus.configure_motion_caps()
    assert v['Profile_Velocity']==[3]*9


def test_teleop_margin_readback_and_exact_restore():
    bus,v=fake({'velocity_counts':14,'acceleration_counts':4})
    v['Drive_Mode']=[0,1,0,1,0,0,1,0,0]
    bus.configure_motion_caps()
    original={k:x[:] for k,x in v.items()}
    result=bus.teleop_speed(True,18,60)
    assert result['readback_verified']
    assert result['velocity_deg_s']==pytest.approx(27.48)
    assert result['acceleration_deg_s2']==pytest.approx(128.7462)
    assert v['Profile_Velocity']==[20]*8+[18]
    assert v['Profile_Acceleration']==[6]*8+[5]
    bus.teleop_speed(False)
    assert v==original


@pytest.mark.parametrize('mode',[4,8,12])
def test_teleop_checks_current_drive_mode_before_writes(mode):
    bus,v=fake(None);v['Drive_Mode']=[mode]*9
    with pytest.raises(ValueError):bus.teleop_speed(True)
    assert v['Profile_Velocity']==[3]*9


def test_teleop_refuses_failed_profile_readback():
    bus,v=fake(None);v['Drive_Mode']=[0]*9
    bus.bus.packet_handler.write4ByteTxRx=lambda *args:(0,0)
    with pytest.raises(ConnectionError,match='readback'):bus.teleop_speed(True)


@pytest.mark.parametrize('speed',[.1,.3,1.])
def test_gripper_profile_follows_speed_without_fixed_twenty_percent_ceiling(speed):
    import math
    from webxr_teleop.hardware.motion import VELOCITY_UNIT_DEG_S
    bus,v=fake(None);bus.gripper_speed=speed
    bus.configure_motion_caps()
    expected=math.ceil(1.5*speed*(1000*360/4096)/VELOCITY_UNIT_DEG_S)
    assert v['Profile_Velocity'][-1]==expected
    assert v['Profile_Velocity'][:8]==[3]*8
    from webxr_teleop.hardware.motion import ACCELERATION_UNIT_DEG_S2
    expected_acc=math.ceil(2*(speed/.3)*(1000*360/4096)/ACCELERATION_UNIT_DEG_S2)
    assert v['Profile_Acceleration']==[1]*8+[expected_acc]
    bus.opening_speed(True);bus.opening_speed(False)
    assert v['Profile_Acceleration'][-1]==expected_acc
    assert v['Profile_Velocity'][-1]==expected


def test_gripper_acceleration_override_is_converted_verified_and_preserved():
    import math
    from webxr_teleop.hardware.motion import ACCELERATION_UNIT_DEG_S2
    bus,v=fake(None);bus.gripper_speed=1.;bus.gripper_acceleration=2.
    bus.configure_motion_caps()
    expected=math.ceil(2*2.*(1000*360/4096)/ACCELERATION_UNIT_DEG_S2)
    assert v['Profile_Acceleration'][-1]==expected
    original={k:x[:] for k,x in v.items()}
    assert bus.gripper_motion_profile['readback_verified']
    bus.opening_speed(True);bus.opening_speed(False)
    assert v==original


def test_unrepresentable_gripper_acceleration_rejected_before_writes():
    bus,v=fake(None);bus.gripper_acceleration=1e8
    with pytest.raises(ValueError,match='gripper acceleration'):bus.configure_motion_caps()
    assert v['Profile_Acceleration']==[1]*9 and v['Profile_Velocity']==[3]*9


def test_gripper_acceleration_readback_failure_is_rejected():
    bus,v=fake(None)
    original_write=bus.bus.packet_handler.write4ByteTxRx
    def wrong_gripper_acceleration(port,idx,address,value):
        if address==108 and idx==8:return 0,0  # Fake successful write with wrong register value.
        return original_write(port,idx,address,value)
    bus.bus.packet_handler.write4ByteTxRx=wrong_gripper_acceleration
    with pytest.raises(ConnectionError,match='Profile_Acceleration readback mismatch'):bus.configure_motion_caps()
