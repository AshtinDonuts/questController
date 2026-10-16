from types import SimpleNamespace
from webxr_teleop.hardware.bus import LeRobotBus
from webxr_teleop.hardware.core import MOTORS


def test_temporary_opening_speed_overrides_slow_cap_and_restores_it():
    bus=LeRobotBus.__new__(LeRobotBus)
    caps=[3]*8+[8];original=caps[:];writes=[];accels=[1]*9
    bus.register=lambda name:(caps if name=='Profile_Velocity' else accels)[:]
    def write(port,idx,address,value):
        assert address in (108,112)
        (caps if address==112 else accels)[idx]=value;writes.append(value);return 0,0
    bus.dxl=SimpleNamespace(COMM_SUCCESS=0)
    bus.bus=SimpleNamespace(motors={n:(i,) for i,n in enumerate(MOTORS)},port_handler=None,
        packet_handler=SimpleNamespace(write4ByteTxRx=write))
    bus.opening_speed(True)
    assert caps==[24]*8+[8]
    assert accels==[5]*8+[1]
    bus.opening_speed(True)
    assert len(writes)==18
    bus.opening_speed(False)
    assert caps==original
    assert accels==[1]*9
    bus.opening_speed(True,counts=6)
    assert caps==[6]*8+[8]
    bus.opening_speed(False)
    assert caps==original
    assert accels==[1]*9
