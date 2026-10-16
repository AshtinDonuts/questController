from copy import deepcopy
import pytest
from webxr_teleop.hardware.check_directions import assess


def sample():return dict(calibrated=[0.]*9,raw=[0]*9)
def arm():return dict(sign=[1]*6,offset_deg=[0]*6)

@pytest.mark.parametrize('delta,expected,sign',[(10,10,1),(-10,10,-1),(10,-10,-1),(-10,-10,1)])
def test_infers_direction_and_preserves_anchor(delta,expected,sign):
    before=sample();before['calibrated'][0]=20
    after=deepcopy(before);after['calibrated'][0]+=delta;after['raw'][0]=100
    result=assess(before,after,0,expected,arm())
    assert result['sign']==sign
    assert (20-result['offset_deg'])/sign==20


def test_other_joint_motion_is_informational():
    a=sample();b=sample();b['calibrated'][0]=10;b['calibrated'][1]=4
    result=assess(a,b,0,10,arm())
    assert result['sign']==1
    assert result['other_joint_deltas']['shoulder']==4


def test_checks_shadow_and_small_motion():
    a=sample();b=sample();b['calibrated'][1]=10;b['calibrated'][2]=-10
    with pytest.raises(ValueError,match='shadow'):assess(a,b,1,10,arm())
    with pytest.raises(ValueError,match='Move'):assess(a,a,0,10,arm())


def test_gripper_reverse_is_reported_not_silently_remapped():
    a=sample();b=sample();b['calibrated'][8]=-15
    result=assess(a,b,6,15,arm())
    assert not result['opening_direction_ok']
    assert 'offset_deg' not in result
