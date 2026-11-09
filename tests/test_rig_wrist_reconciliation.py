"""Regression anchors from the operator-observed opening and sleep tests."""
import json
import math
from pathlib import Path

import pytest

from webxr_teleop.hardware.core import JOINTS, load_profile, validate_sequence

ROOT = Path(__file__).resolve().parents[1]


def configuration():
    profile = load_profile(ROOT / 'webxr_teleop/hardware/rig.json')
    calibration = json.loads((ROOT / 'webxr_teleop/hardware/calibration' /
                              Path(profile['arms']['left']['calibration']).name).read_text())
    return profile, calibration


@pytest.mark.parametrize('pose,expected', [
    ('opening', [2116, 1407, 2580, 2049, 1704, 2083]),
    ('sleep', [2045, 811, 3122, 2020, 3555, 2063]),
])
def test_normal_rig_uses_selected_opening_and_preserves_captured_sleep(pose, expected):
    profile, calibration = configuration()
    validate_sequence(profile)
    arm = profile['arms']['left']
    raw = []
    for j, name in enumerate(JOINTS):
        i = calibration['motor_names'].index(name)
        calibrated = math.degrees(arm[pose + '_rad'][j]) * arm['sign'][j] + arm['offset_deg'][j]
        count = calibrated * 4096 / 360 - calibration['homing_offset'][i]
        raw.append(round(-count if calibration['drive_mode'][i] else count))
    assert raw == expected


def test_operator_selected_wrist_alignment_and_direction():
    profile, calibration = configuration()
    arm = profile['arms']['left']
    for joint, count, expected in [('wrist_angle', 2651, -29.998046875),
                                    ('wrist_rotate', 2074, .17578125)]:
        j = JOINTS.index(joint)
        i = calibration['motor_names'].index(joint)
        calibrated = ((-count if calibration['drive_mode'][i] else count)
                      + calibration['homing_offset'][i]) * 360 / 4096
        assert (calibrated - arm['offset_deg'][j]) / arm['sign'][j] == pytest.approx(expected)
    assert arm['sign'][JOINTS.index('wrist_angle')] == -1


def test_restored_opening_angle_stays_within_calibrated_decoder_range():
    profile, _ = configuration()
    arm = profile['arms']['left']
    assert math.degrees(arm['opening_rad'][4]) == pytest.approx(53.26171875)
    for model in (arm['opening_rad'][4], arm['sleep_rad'][4],
                  arm['lower_rad'][4], arm['upper_rad'][4]):
        calibrated = math.degrees(model) * arm['sign'][4] + arm['offset_deg'][4]
        assert -270 <= calibrated <= 270
