"""Run under xvfb-run: exercise the actual passive viewer without a motor bus."""
import json
import multiprocessing as mp
import os
import time
import pytest
from webxr_teleop.hardware.ipc import pair,send
from webxr_teleop.hardware.run import DEFAULT_PROFILE
from webxr_teleop.hardware.twin import run


@pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='requires an X display (use xvfb-run)')
def test_measured_twin_opens_updates_and_exits():
    profile=json.loads(DEFAULT_PROFILE.read_text())
    parent,child=pair()
    proc=mp.get_context('spawn').Process(target=run,args=(child,profile,('left',)))
    proc.start();child.close()
    q=profile['arms']['left']['initial_rad']+[.2]
    try:
        for i in range(60):
            send(parent,dict(t=time.monotonic(),feedback_fresh=i<45,measured={'left':q},
                command={'diagnostics':{'left':dict(desired_position=[0,.3,.4],filtered_position=[0,.3,.3])}}))
            time.sleep(.05)
            assert proc.is_alive(),f'Viewer exited unexpectedly: {proc.exitcode}'
        send(parent,dict(op='shutdown'));proc.join(5)
        assert proc.exitcode==0
    finally:
        if proc.is_alive():proc.terminate();proc.join(3)
        parent.close()
