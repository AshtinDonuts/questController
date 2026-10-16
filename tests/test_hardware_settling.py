import math
from webxr_teleop.hardware.core import SettlingWindow

TICK=2*math.pi/4096


def test_encoder_tick_jitter_settles():
    window=SettlingWindow(); target=[0.]*7
    for i in range(31):
        q=[TICK*(i%2)]*6+[0.]
        result=window.update(q,target,i*.02)
        if i<15: assert not result
    assert result


def test_cumulative_drift_does_not_settle():
    window=SettlingWindow()
    for i in range(100):
        assert not window.update([i*.0003]*6+[0.],[0.]*7,i*.02)


def test_gap_changed_target_and_cached_sample_do_not_settle():
    window=SettlingWindow(); q=[0.]*7
    for i in range(20): window.update(q,q,i*.02)
    assert not window.update(q,q,.38)
    assert not window.update(q,q,.6)
    assert not window.update(q,[.001]*6+[0.],.62)


def test_stable_but_far_from_goal_does_not_settle():
    window=SettlingWindow()
    for i in range(30):
        assert not window.update([.02]*6+[0.],[0.]*7,i*.02)
