"""Offline pose conversion; no LeRobot imports or hardware access."""
import copy
import hashlib
import json
import math
from pathlib import Path

import pytest
from webxr_teleop.hardware.core import JOINTS, SIDES, load_profile, validate_sequence
from webxr_teleop.hardware.import_poses import import_sequence
from webxr_teleop.hardware.run import DEFAULT_PROFILE


def fixture(tmp_path):
    profile = load_profile(DEFAULT_PROFILE)
    for arm in profile['arms'].values():
        arm['sign'] = [-1, -1, 1, -1, 1, -1]
        arm['offset_deg'] = [-1.7578125, 90, 88.4375, 6.6796875, 0, -2.109375]
    document = {'robot_type': 'aloha', 'joint_values': {}}
    for side in SIDES:
        arm = profile['arms'][side]
        values = {name: math.degrees(q)*sign+offset for name,q,sign,offset in
                  zip(JOINTS,arm['home_rad'],arm['sign'],arm['offset_deg'])}
        # Arbitrary shadow/gripper values must never shift primary joint indexing.
        values.update(shoulder_shadow=999, elbow_shadow=888, gripper=73)
        names = list(reversed(values))
        document['joint_values'][side+'_follower'] = dict(
            motor_names=names, positions=[values[n] for n in names], positions_dict=values)
    path = tmp_path/'pose.json'
    path.write_text(json.dumps(document))
    return profile, path, document


def test_import_by_joint_name_roundtrips_updated_mapping_and_preserves_input(tmp_path):
    profile,path,_ = fixture(tmp_path)
    original = copy.deepcopy(profile)
    result = import_sequence(profile,path,path)
    assert profile == original
    validate_sequence(result)
    for side in SIDES:
        arm=result['arms'][side]
        for pose in ('opening','sleep'):
            assert arm[pose+'_rad'] == pytest.approx(arm['home_rad'])
            source=arm['sequence_sources'][pose]
            assert source['sha256']==hashlib.sha256(path.read_bytes()).hexdigest()
            for name,q,sign,offset in zip(JOINTS,arm[pose+'_rad'],arm['sign'],arm['offset_deg']):
                assert math.degrees(q)*sign+offset == pytest.approx(source['calibrated_degrees'][name])
        assert not arm['sequence_verified']
        assert arm['initial_gripper']==original['arms'][side]['initial_gripper']


@pytest.mark.parametrize('fault', ['duplicate','missing','disagreement','nonfinite','robot'])
def test_invalid_source_rejected(tmp_path,fault):
    profile,path,document=fixture(tmp_path)
    record=document['joint_values']['left_follower']
    if fault=='duplicate': record['motor_names'][0]=record['motor_names'][1]
    elif fault=='missing': record['motor_names'].remove('waist');record['positions'].pop()
    elif fault=='disagreement': record['positions_dict']['waist']+=1
    elif fault=='nonfinite': record['positions'][0]=float('nan')
    else: document['robot_type']='other'
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError): import_sequence(profile,path,path)


def test_mapping_change_requires_reimport(tmp_path):
    profile,path,_=fixture(tmp_path)
    result=import_sequence(profile,path,path)
    result['arms']['left']['offset_deg'][0]+=1
    with pytest.raises(ValueError,match='mapping changed'):
        validate_sequence(result)


def test_rig_targets_match_recorded_aloha_sources_without_clamping():
    profile=load_profile(Path(DEFAULT_PROFILE).with_name('rig.json'))
    for side in SIDES:
        arm=profile['arms'][side]
        for pose,filename in [('opening','ceremony_pose.json'),('sleep','rest_pose.json')]:
            source=arm['sequence_sources'][pose]
            assert Path(source['path']).name==filename
            assert source['arm']==side+'_follower'
            assert source['sign']==arm['sign'] and source['offset_deg']==arm['offset_deg']
            for name,q,sign,offset in zip(JOINTS,arm[pose+'_rad'],arm['sign'],arm['offset_deg']):
                assert math.degrees(q)*sign+offset==pytest.approx(source['calibrated_degrees'][name])


def test_operator_authorization_is_separate_from_verification_and_import_clears_it(tmp_path):
    profile,path,_=fixture(tmp_path)
    for arm in profile['arms'].values():
        arm['sequence_authorized']=True
        arm['sequence_authorization_note']='Explicit test authorization'
    validate_sequence(profile,live=True)
    result=import_sequence(profile,path,path)
    assert all(not a['sequence_authorized'] and not a['sequence_verified'] for a in result['arms'].values())
    with pytest.raises(ValueError,match='authorization'):
        validate_sequence(result,live=True)
