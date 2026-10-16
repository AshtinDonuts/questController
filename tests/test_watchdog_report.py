from copy import deepcopy
from webxr_teleop.hardware.test_watchdog import evaluate

def fixture():
    before=dict(raw=[100]*9,goal=[100]*9)
    sample=dict(raw=[100]*9,goal=[100]*9,torque=[1]*9,watchdog=[255]*9,errors=[0]*9,velocity=[0]*9)
    return before,sample

def test_stationary_powered_latched_hold_passes():
    b,s=fixture();assert evaluate(b,[s])['passed']

def test_drift_torque_and_timeout_failures_rejected():
    for key,value in [('raw',[106]*9),('goal',[101]*9),('torque',[0]*9),('watchdog',[15]*9),('errors',[1]*9),('velocity',[2]*9)]:
        b,s=fixture();s[key]=value
        assert not evaluate(b,[s])['passed'],key
    b,s=fixture();assert not evaluate(b,[])['passed']
