"""User-measured mounting must agree in hardware FK, twin model and simulation."""
import mujoco
import numpy as np
import pytest

from webxr_teleop.hardware.core import SIDES, load_profile
from webxr_teleop.hardware.planner import make_model, put_measured
from webxr_teleop.hardware.run import ROOT, DEFAULT_PROFILE


def assert_mounting(model,data):
    left=model.body('left/base_link').id
    right=model.body('right/base_link').id
    np.testing.assert_allclose(data.xpos[left],[-.4075,0,.02],atol=1e-10)
    np.testing.assert_allclose(data.xpos[right],[.4075,0,.02],atol=1e-10)
    np.testing.assert_allclose(data.xpos[right]-data.xpos[left],[.815,0,0],atol=1e-10)
    for base in (left,right):
        forward=data.xmat[base].reshape(3,3)@np.array([1.,0,0])
        np.testing.assert_allclose(forward,[0,1.,0],atol=1e-9)
    # Equal measured joint poses yield equally oriented EEFs separated only by
    # the measured base translation: catches mirroring/rotation mistakes.
    lsite=model.site('left/gripper').id;rsite=model.site('right/gripper').id
    np.testing.assert_allclose(data.site_xpos[rsite]-data.site_xpos[lsite],[.815,0,0],atol=1e-9)
    np.testing.assert_allclose(data.site_xmat[rsite],data.site_xmat[lsite],atol=1e-9)


@pytest.mark.parametrize('name',['rig.json','example_profile.json'])
def test_profile_mounting_reaches_measured_fk_and_twin(name):
    profile=load_profile(DEFAULT_PROFILE.with_name(name))
    assert profile['provenance']['mounting_geometry']['base_center_separation_m']==.815
    measured={s:profile['arms'][s]['initial_rad']+[.5] for s in SIDES}
    model=make_model(profile,SIDES);data=mujoco.MjData(model)
    put_measured(model,data,profile,measured)
    assert_mounting(model,data)
    if name=='rig.json':
        assert profile['arms']['right']['verified']==profile['arms']['left']['verified']
        assert profile['provenance']['right_mapping_status_copy']['independent_right_physical_test'] is False
        assert profile['arms']['left']['gripper_current_cap_counts']==200


def test_standalone_dual_simulator_matches_hardware_mounting():
    model=mujoco.MjModel.from_xml_path(str(ROOT/'scenes/dual_viperx_scene.xml'))
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    assert_mounting(model,data)


def test_new_recording_replay_restores_recorded_mounting(tmp_path):
    import json
    from scipy.spatial.transform import Rotation
    from webxr_teleop.replay_ik import load_rows, replay
    model=mujoco.MjModel.from_xml_path(str(ROOT/'scenes/dual_viperx_scene.xml'))
    data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data)
    row=dict(elapsed_s=0.,qpos=data.qpos.tolist(),arms={})
    for side in SIDES:
        site=model.site(side+'/gripper').id
        row['arms'][side]=dict(target_position=data.site_xpos[site].tolist(),
            target_orientation_xyzw=Rotation.from_matrix(data.site_xmat[site].reshape(3,3)).as_quat().tolist())
    source=tmp_path/'telemetry.jsonl';source.write_text(json.dumps(row)+'\n')
    (tmp_path/'manifest.json').write_text(json.dumps(dict(scene='dual',base_positions={
        'left':[-.4075,0.,.02],'right':[.4075,0.,.02]})))
    frames=load_rows(source)
    model,data,frame=next(replay(frames))
    assert_mounting(model,data)
    assert all(s['accepted'] and s['ik_position_error_m']<1e-8 for s in frame['arms'].values())
