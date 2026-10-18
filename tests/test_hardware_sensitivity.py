import copy
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from webxr_teleop.hardware.core import load_profile,teleop_scales,SIDES
from webxr_teleop.hardware.run import DEFAULT_PROFILE
from webxr_teleop.hardware.planner import Planner


@pytest.mark.parametrize('translation,rotation',[(1.,1.),(.5,2.),(2.,.25),(0.,1.),(1.,0.)])
def test_independent_gains_and_fresh_grip_reference(monkeypatch,translation,rotation):
    p=load_profile(DEFAULT_PROFILE);p.update(translation_scale=translation,rotation_scale=rotation)
    q={s:p['arms'][s]['initial_rad']+[p['arms'][s]['initial_gripper']] for s in SIDES}
    h={s:dict(position=[0.,1.2,-.4],quaternion=[0.,0.,0.,1.],grip=False,trigger=0.) for s in SIDES}
    clock=[0.];monkeypatch.setattr('webxr_teleop.hardware.planner.time.monotonic',lambda:clock[0])
    planner=Planner(p,q)
    def step(mode='teleop'):
        clock[0]+=.02
        return planner.step(dict(t=clock[0],seq=int(clock[0]*1000),epoch=1,mode=mode,measured=q,hands=h))
    step('align');h['left']['grip']=True;step()
    origin=copy.deepcopy(planner.origin['left'])
    delta=np.array([.01,.02,-.01]);angle=.1
    h['left']['position']=(origin['hand']+delta).tolist()
    h['left']['quaternion']=Rotation.from_rotvec([0,angle,0]).as_quat().tolist()
    step()
    current=planner.origin['left']
    np.testing.assert_allclose(current['target_p']-origin['p'],translation*(planner.basis@delta),atol=1e-12)
    actual=Rotation.from_matrix(current['target_r']@origin['r'].T).as_rotvec()
    np.testing.assert_allclose(actual,rotation*(planner.basis@np.array([0,angle,0])),atol=1e-12)
    h['left']['grip']=False;step()
    h['left']['position']=[1.,1.,1.];h['left']['quaternion']=Rotation.from_euler('xyz',[.3,.2,.6]).as_quat().tolist()
    step();h['left']['grip']=True;step()
    new=planner.origin['left']
    np.testing.assert_allclose(new['target_p'],new['p'],atol=1e-12)
    np.testing.assert_allclose(new['target_r'],new['r'],atol=1e-12)


@pytest.mark.parametrize('key',['translation_scale','rotation_scale'])
@pytest.mark.parametrize('value',[-1,math.inf,math.nan,True,'2'])
def test_invalid_gains_rejected(key,value):
    with pytest.raises(ValueError,match=key):teleop_scales({key:value})


def test_default_gains_preserve_one_to_one_mapping():
    assert teleop_scales({})==(1.,1.)
