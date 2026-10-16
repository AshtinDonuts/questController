"""Offline import of calibrated ALOHA pose JSONs; never imports a motor driver."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from .core import JOINTS, SIDES, vector


def import_sequence(profile, opening, sleep):
    result = copy.deepcopy(profile)
    for pose_name, source in (('opening', opening), ('sleep', sleep)):
        path = Path(source).resolve()
        raw = path.read_bytes()
        document = json.loads(raw)
        if document.get('robot_type') != 'aloha':
            raise ValueError(f'{path}: expected aloha pose')
        for side in SIDES:
            arm = result['arms'][side]
            record = document['joint_values'][side+'_follower']
            names, positions = record['motor_names'], record['positions']
            vector(positions, len(names))
            if len(set(names)) != len(names):
                raise ValueError('Duplicate motor names in pose')
            named = dict(zip(names, positions))
            if any(name not in named for name in JOINTS):
                raise ValueError(f'{side}: missing primary joints in pose')
            if record.get('positions_dict', named) != named:
                raise ValueError('Pose positions and positions_dict disagree')
            # LeRobot calibrated degrees = degrees(model radians) * sign + offset.
            vector(arm['sign'], 6); vector(arm['offset_deg'], 6)
            if any(sign not in (-1, 1) for sign in arm['sign']):
                raise ValueError('Mapping signs must be -1 or +1')
            arm[pose_name+'_rad'] = [math.radians((named[joint]-offset)/sign)
                for joint, sign, offset in zip(JOINTS, arm['sign'], arm['offset_deg'])]
            arm.setdefault('sequence_sources', {})[pose_name] = dict(
                path=str(path), sha256=hashlib.sha256(raw).hexdigest(),
                arm=side+'_follower', calibrated_degrees={name:named[name] for name in JOINTS},
                sign=arm['sign'][:], offset_deg=arm['offset_deg'][:],
                gripper_policy='Preserve current opening; source gripper is not commanded')
            arm['sequence_authorized'] = False
            arm['sequence_authorization_note'] = ''
            arm['sequence_verified'] = False
            arm['sequence_verification_note'] = 'Imported ALOHA targets; model limits, geometry and physical holding still require verification.'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--opening', type=Path, required=True)
    parser.add_argument('--sleep', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = import_sequence(json.loads(args.profile.read_text()), args.opening, args.sleep)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(f'Imported exact targets to {args.output}; no hardware accessed, no verification flags granted.')


if __name__ == '__main__':
    main()
