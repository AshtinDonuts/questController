import json
import numpy as np
import pytest
from webxr_teleop.hardware import bus,mirror
from webxr_teleop.hardware.core import load_profile
from webxr_teleop.hardware.run import DEFAULT_PROFILE

@pytest.mark.parametrize('torque',[0,1])
def test_mirror_only_reads_and_refuses_powered_motors(monkeypatch,capsys,torque):
    events=[]
    class FakeBus:
        def __init__(self,*args):
            self.bus=self;self.inspect()
        def register(self,name):
            events.append(name)
            return {'Model_Number':[1120]*7+[1020]*2,'Torque_Enable':[torque]*9,'Operating_Mode':[4]*8+[5],'Hardware_Error_Status':[0]*9}[name]
        def read_raw(self):events.append('position');return np.zeros(9,dtype=np.int32)
        def apply_calibration(self,raw,names):return raw.astype(float)
        def close(self):events.append('close')
        def write(self,*args):raise AssertionError('Motor write forbidden')
    monkeypatch.setattr(bus,'LeRobotBus',FakeBus)
    monkeypatch.setattr(mirror.signal,'signal',lambda *a:None)
    def end(_):raise KeyboardInterrupt
    monkeypatch.setattr(mirror.time,'sleep',end)
    if torque:
        with pytest.raises(ValueError,match='already enabled'):
            mirror.reader(load_profile(DEFAULT_PROFILE),'left')
        assert 'position' not in events
    else:
        mirror.reader(load_profile(DEFAULT_PROFILE),'left')
        assert json.loads(capsys.readouterr().out)['torque']==[0]*9
        assert events[-1]=='close'
