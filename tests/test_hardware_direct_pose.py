import json
import pytest
from webxr_teleop.hardware.core import MOTORS
from webxr_teleop.hardware.direct_pose import pose_values


def test_original_calibrated_values_are_selected_by_name(tmp_path):
    names=list(reversed(MOTORS)); positions=list(range(9))
    p=tmp_path/'pose.json';p.write_text(json.dumps(dict(robot_type='aloha',joint_values={'left_follower':dict(motor_names=names,positions=positions)})))
    assert pose_values(p,'left')==list(reversed(positions))


def test_inconsistent_pose_refused(tmp_path):
    p=tmp_path/'pose.json';p.write_text(json.dumps(dict(robot_type='aloha',joint_values={'left_follower':dict(motor_names=list(MOTORS),positions=[0]*9,positions_dict={})})))
    with pytest.raises(ValueError,match='disagree'):pose_values(p,'left')
