"""The captured action comes from motor goal readback, after quantization."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from webxr_teleop.hardware.bus import LeRobotBus
from webxr_teleop.hardware.core import JOINTS


def test_capture_uses_final_raw_readback_and_keeps_confirmation_for_unchanged_goal():
    bus=LeRobotBus.__new__(LeRobotBus)
    bus.np=np
    bus.record_actions=True
    bus._gripper_current_cap={}
    bus.cfg=dict(sign=[1,-1,1,-1,1,-1],offset_deg=[10]*6)
    bus.gripper_calibration=dict(closed_raw=100,open_raw=200)
    bus.command_names=list(JOINTS)+['gripper']
    raw=[1000,1100,1200,1300,1400,1500,150]
    bus.raw_target=lambda q:raw
    calls=[]
    class Writer:
        def addParam(self,*args):pass
        def txPacket(self):calls.append('write');return 0
    bus.dxl=SimpleNamespace(COMM_SUCCESS=0,GroupSyncWrite=lambda *args:Writer())
    bus.bus=SimpleNamespace(port_handler=None,packet_handler=None,
        motors={name:(index,) for index,name in enumerate(bus.command_names)},
        revert_calibration=lambda *args:[1100,1200],
        apply_calibration=lambda *args:np.array([11,12,13,14,15,16,50]))
    bus.register=lambda *args:raw
    assert bus.write([.01]*6+[.5])
    action=bus.confirmed_action.copy()
    assert action['raw_goal']==raw
    expected=[math.radians((v-10)/s) for v,s in zip([11,12,13,14,15,16],bus.cfg['sign'])]+[.5]
    assert action['qpos']==pytest.approx(expected)
    assert action['qpos']!=[.01]*6+[.5]
    assert bus.write([.01]*6+[.5]) is False
    assert bus.confirmed_action==action
    assert calls==['write']
