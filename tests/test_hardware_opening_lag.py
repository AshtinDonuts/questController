import math
import pytest
from webxr_teleop.hardware.core import Guard


def guard(sequence=True):
    g=Guard([-3.]*6,[3.]*6,sequence=sequence);g.arm([0.]*7,1)
    return g


def test_scripted_opening_accepts_lag_over_time():
    g=guard();measured=[-math.radians(3)]*6+[0.]
    for i in range(60):
        now=i*.02
        g.command(dict(epoch=1,seq=i,issued=now,q=[0.]*7,mode='opening'),measured,now)
        assert g.state=='ARMED'
    g.tick(measured,2.)
    assert g.state=='HOLD' and 'watchdog' in g.reason.lower()


@pytest.mark.parametrize('mode,sequence',[('teleop',True),('sleep',True),('opening',False)])
def test_tracking_lag_does_not_disarm(mode,sequence):
    g=guard(sequence)
    g.command(dict(epoch=1,seq=1,issued=0.,q=[0.]*7,mode=mode),[-math.radians(3)]*6+[0.],0.)
    assert g.state=='ARMED'


@pytest.mark.parametrize('mode',['teleop','sleep'])
def test_persistent_lag_does_not_disarm(mode):
    g=guard()
    for i in range(30):
        g.command(dict(epoch=1,seq=i,issued=i*.02,q=[0.]*7,mode=mode),[-math.radians(1.5)]*6+[0.],i*.02)
    assert g.state=='ARMED'
