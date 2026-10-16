import math
import pytest
from test_hardware_planner import setup
from webxr_teleop.hardware.planner import Planner


@pytest.mark.parametrize('mode,error_deg,expected',[('opening',1.7,True),('opening',22,False),('sleep',1.7,False)])
def test_arrival_requires_proximity_and_stationarity(monkeypatch,mode,error_deg,expected):
    p,q,_=setup()
    for s in q:
        q[s]=p['arms'][s][mode+'_rad'][:]+[.8]
        q[s][0]+=math.radians(error_deg)
    planner=Planner(p,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    done=False
    for i in range(600 if mode=='opening' and error_deg>2 else 175):
        clock[0]=i*.02
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode=mode,home_id=1,measured=q,hands={}))
        done|=result['home_done']
    assert done==expected


def test_opening_waits_past_old_timeout_then_completes(monkeypatch):
    p,q,_=setup()
    for s in q:
        q[s]=p['arms'][s]['opening_rad'][:]+[.8]
        q[s][0]+=.06
    planner=Planner(p,q);clock=[0.]
    monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    for i in range(450):
        clock[0]=i*.02
        if i>350:
            for s in q:q[s][0]=p['arms'][s]['opening_rad'][0]+math.radians(1.7)+(i%2)*2*math.pi/4096
        result=planner.step(dict(t=clock[0],seq=i,epoch=1,mode='opening',home_id=1,measured=q,hands={}))
        if i<=350:assert not result['home_done']
    assert result['home_done']
