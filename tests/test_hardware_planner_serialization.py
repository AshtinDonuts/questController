import time
import numpy as np
from webxr_teleop.hardware import planner
from webxr_teleop.hardware.ipc import pair,receive


def test_serialization_failure_keeps_planner_available_for_sleep(monkeypatch):
    parent,child=pair();now=time.monotonic()
    jobs=iter([[dict(t=now,seq=1,epoch=1,mode='teleop',measured={})],
               [dict(t=now,seq=2,epoch=2,mode='sleep',measured={})],
               [dict(op='shutdown')]])
    class FakePlanner:
        def __init__(self,*args):pass
        def step(self,job):
            return dict(epoch=job['epoch'],mode=job['mode'],
                        diagnostic=np.bool_(True) if job['mode']=='teleop' else True)
    monkeypatch.setattr(planner,'Planner',FakePlanner)
    monkeypatch.setattr(planner,'receive',lambda sock:next(jobs))
    try:
        planner.run(child,{},('left',))
        messages=receive(parent)
        assert messages[0]['type']=='fault'
        assert messages[0]['recoverable'] is True
        assert messages[0]['code']=='serialization'
        assert {k:messages[1][k] for k in ('type','epoch','mode','diagnostic')}==dict(type='target',epoch=2,mode='sleep',diagnostic=True)
        assert messages[1]['planner_duration_ms']>=0
        assert messages[1]['planner_queue_age_ms']>=0
    finally:parent.close();child.close()
