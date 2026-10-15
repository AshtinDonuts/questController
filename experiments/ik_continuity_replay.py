"""Offline diagnostic: replay recorded EEF targets with regularized, rate-bounded IK.
Does not change the live controller. Run from repository root with PYTHONPATH=.
"""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from webxr_teleop.control import Teleop
from webxr_teleop.recording import Recording

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('source', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
rows = [json.loads(l) for l in (args.source/'telemetry.jsonl').read_text().splitlines()]
model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'scenes/dual_viperx_scene.xml'))
data = mujoco.MjData(model)
data.qpos[:] = rows[0]['qpos']
mujoco.mj_forward(model, data)
arms = {s: Teleop(model, data, side=s) for s in ('left', 'right')}
recording = Recording(args.output, model, 30, dict(experiment='regularized IK target replay',
    source=str(args.source.resolve()), joint_speed_deg_s=60, regularization_weight=.02))
metrics = []
status = 'failed'
try:
    for row in rows:
        states = {}
        for side, arm in arms.items():
            desired = row['arms'][side]
            p = np.array(desired['target_position'])
            r = R.from_quat(desired['target_orientation_xyzw']).as_matrix()
            current = data.qpos[arm.qids].copy()
            arm.scratch.qpos[:] = data.qpos
            def residual(q):
                arm.scratch.qpos[arm.qids] = q
                mujoco.mj_forward(model, arm.scratch)
                pos = arm.scratch.site_xpos[arm.site]
                rot = arm.scratch.site_xmat[arm.site].reshape(3,3)
                return np.r_[pos-p, .25*R.from_matrix(r@rot.T).as_rotvec(), .02*(q-current)]
            step = np.deg2rad(60)/30
            result = least_squares(residual, current, bounds=(np.maximum(arm.bounds[0],current-step),
                np.minimum(arm.bounds[1],current+step)), max_nfev=35, ftol=1e-7, xtol=1e-7, gtol=1e-7)
            data.qpos[arm.qids] = result.x
            data.ctrl[arm.aids] = result.x
            data.qpos[arm.fingers] = np.array(row['qpos'])[arm.fingers]
            mujoco.mj_forward(model,data)
            actual_p, actual_r = arm.actual_pose()
            states[side] = dict(position_mm=float(1000*np.linalg.norm(p-actual_p)),
                rotation_deg=float(np.rad2deg(np.linalg.norm(R.from_matrix(r@actual_r.T).as_rotvec()))),
                speed_deg_s=float(np.max(np.abs(np.rad2deg(result.x-current)))*30))
        metrics.append(dict(time=row['elapsed_s'], **states))
        data.time = row['elapsed_s']
        recording.write(data, dict(elapsed_s=row['elapsed_s'], qpos=data.qpos.tolist(), errors=states))
    status = 'completed'
finally:
    recording.close(status)
summary = {s:{k:max(r[s][k] for r in metrics) for k in ('position_mm','rotation_deg','speed_deg_s')} for s in arms}
(args.output/'comparison.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps(summary,indent=2))
