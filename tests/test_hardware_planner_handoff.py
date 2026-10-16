import math
import pytest
from webxr_teleop.hardware.core import Guard

TICK=2*math.pi/4096


def make_guard():
    g=Guard([-3.]*6,[3.]*6,sequence=True);g.arm([0.]*7,1)
    return g


def command(q,seq=1,**kwargs):
    return dict(epoch=1,seq=seq,issued=seq*.02,q=q,mode='opening',trajectory_id=1,**kwargs)


def test_planner_origin_wins_over_one_tick_worker_difference():
    g=make_guard();q=[0.]*7;q[5]=TICK
    assert g.command(command(q,trajectory_start=True),[0.]*7,.02)==q
    assert g.state=='ARMED' and g.velocity==[0.]*7
    q=q[:];q[5]+=1e-6
    g.command(command(q,seq=2),q,.04)
    assert g.state=='ARMED'
    # Worker does not re-differentiate planner targets on its own clock.
    q=q[:];q[5]+=.001
    g.command(command(q,seq=3),q,.06)
    assert g.state=='ARMED'


@pytest.mark.parametrize('kind',['large','repeated','moving'])
def test_handoff_does_not_allow_unbounded_or_repeated_rebase(kind):
    g=make_guard();q=[0.]*7
    if kind=='large':q[0]=3*TICK
    elif kind=='moving':g.velocity[0]=.001
    else:
        g.command(command(q,trajectory_start=True),q,.02)
    g.command(command(q,seq=2,trajectory_start=True),q,.04)
    assert g.state=='HOLD'
