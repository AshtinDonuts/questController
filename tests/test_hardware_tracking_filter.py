import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from webxr_teleop.hardware.tracking import PoseDeadband, tracking_deadbands


def rotation(degrees):
    return Rotation.from_euler('z',degrees,degrees=True).as_matrix()


def test_jitter_holds_position_and_rotation_without_drift():
    filt=PoseDeadband(np.zeros(3),np.eye(3),.002,math.radians(1))
    for i in range(100):
        p,r=filt.update([(-1)**i*.001,0,0],rotation((-1)**i*.8))
        np.testing.assert_array_equal(p,np.zeros(3))
        np.testing.assert_array_equal(r,np.eye(3))


def test_slow_motion_accumulates_and_crosses_threshold_continuously():
    filt=PoseDeadband(np.zeros(3),np.eye(3),.002,math.radians(1))
    for i in range(1,11):
        p,r=filt.update([i*.001,0,0],rotation(i*.5))
        np.testing.assert_allclose(p,[max(0,i*.001-.002),0,0],atol=1e-12)
        np.testing.assert_allclose(r,rotation(max(0,i*.5-1)),atol=1e-12)
    for _ in range(10):
        # A stationary controller must not cause the filter to catch up further.
        p,r=filt.update([.01,0,0],rotation(5))
        np.testing.assert_allclose(p,[.008,0,0],atol=1e-12)
        np.testing.assert_allclose(r,rotation(4),atol=1e-12)


def test_threshold_is_radial_and_position_rotation_are_independent():
    filt=PoseDeadband(np.zeros(3),np.eye(3),.002,math.radians(1))
    p,r=filt.update([.002,.002,0],rotation(.5))
    expected=np.array([.002,.002,0])*(1-.002/math.hypot(.002,.002))
    np.testing.assert_allclose(p,expected)
    np.testing.assert_array_equal(r,np.eye(3))
    p2,r=filt.update(p,rotation(3))
    np.testing.assert_array_equal(p2,p)
    np.testing.assert_allclose(r,rotation(2),atol=1e-12)


def test_rotation_wrap_uses_shortest_path():
    filt=PoseDeadband(np.zeros(3),rotation(179.5),.002,math.radians(1))
    _,r=filt.update([0,0,0],rotation(-179.5))
    np.testing.assert_allclose(r,rotation(179.5),atol=1e-12)
    _,r=filt.update([0,0,0],rotation(-178))
    np.testing.assert_allclose(r,rotation(-179),atol=1e-12)


def test_zero_thresholds_preserve_input():
    filt=PoseDeadband(np.zeros(3),np.eye(3),0,0)
    p,r=filt.update([.001,0,0],rotation(.1))
    np.testing.assert_allclose(p,[.001,0,0])
    np.testing.assert_allclose(r,rotation(.1),atol=1e-12)


@pytest.mark.parametrize('key',['tracking_position_deadband_m','tracking_rotation_deadband_deg'])
@pytest.mark.parametrize('value',[-1,math.inf,math.nan,True,'2'])
def test_invalid_thresholds_rejected(key,value):
    with pytest.raises(ValueError,match=key):tracking_deadbands({key:value})


def test_motion_gate_bridges_samples_and_counts_slow_translation_and_rotation():
    from webxr_teleop.hardware.tracking import PoseMotionGate
    gate=PoseMotionGate(np.zeros(3),np.eye(3),0.)
    assert not gate.moving(np.zeros(3),np.eye(3),.09)
    assert not gate.moving(np.zeros(3),np.eye(3),.11)
    # Changes smaller than the activity threshold accumulate against its anchor.
    for i in range(1,6):
        moving=gate.moving(np.array([i*.00011,0,0]),np.eye(3),.11+i*.01)
    assert moving
    assert gate.moving(np.array([.00055,0,0]),np.eye(3),.24)
    assert not gate.moving(np.array([.00055,0,0]),np.eye(3),.27)
    assert gate.moving(np.array([.00055,0,0]),rotation(.25),.28)
