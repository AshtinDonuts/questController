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
    assert v['Profile_Velocity']==[14]*8+[3]
    assert v['Profile_Acceleration']==[4]*8+[1]
    bus.opening_speed(True);assert v['Profile_Velocity']==[24]*8+[3]
    bus.opening_speed(False);assert v['Profile_Velocity']==[14]*8+[3]


def test_untuned_caps_remain_conservative():
    bus,v=fake(None);bus.configure_motion_caps()
    assert v['Profile_Velocity']==[3]*9
    assert v['Profile_Acceleration']==[1]*9


@pytest.mark.parametrize('value',[0,-1,True,1.5,32768])
def test_invalid_profile_refused_before_writes(value):
    bus,v=fake({'velocity_counts':value,'acceleration_counts':4})
    with pytest.raises(ValueError):bus.configure_motion_caps()
    assert v['Profile_Velocity']==[3]*9


def test_time_based_mode_refused():
    bus,v=fake({'velocity_counts':14,'acceleration_counts':4});bus.preflight['Drive_Mode'][0]=4
    with pytest.raises(ValueError,match='velocity-based'):bus.configure_motion_caps()
    assert v['Profile_Velocity']==[3]*9
