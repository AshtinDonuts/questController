"""Read-only, torque-off encoder mirror. No IK, target writes, or torque changes."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from .core import JOINTS, MOTORS, load_profile


def reader(profile, side):
    from .bus import LeRobotBus

    class FeedbackOnly(LeRobotBus):
        def inspect(self):
            result={n:self.register(n) for n in ('Model_Number','Torque_Enable','Operating_Mode','Hardware_Error_Status')}
            if result['Model_Number'] != [1120]*7+[1020]*2:
                raise ValueError('Unexpected follower motor identities')
            if any(result['Torque_Enable']):
                raise ValueError('Torque is already enabled; refusing torque-off diagnostic without changing it')
            return result

    bus=None
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:
        bus=FeedbackOnly(profile,side)
        while True:
            raw=bus.read_raw()
            torque=bus.register('Torque_Enable')
            if any(torque):
                raise RuntimeError('Torque became enabled; ending read-only diagnostic')
            values=bus.bus.apply_calibration(raw.copy(),list(MOTORS))
            a=profile['arms'][side]
            q=[math.radians((float(values[list(MOTORS).index(n)])-a['offset_deg'][i])/a['sign'][i]) for i,n in enumerate(JOINTS)]
            q.append(float(values[-1])/100)
            print(json.dumps(dict(t=time.monotonic(),q=q,raw=raw.tolist(),calibrated=values.tolist(),torque=torque)),flush=True)
            time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        if bus: bus.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--side',choices=('left','right'),default='left')
    ap.add_argument('--profile',type=Path,default=Path(__file__).with_name('rig.json'))
    ap.add_argument('--lerobot-python',default='/home/King/miniconda3/envs/lerobot/bin/python')
    ap.add_argument('--reader',action='store_true',help=argparse.SUPPRESS)
    ap.add_argument('--headless',action='store_true')
    ap.add_argument('--seconds',type=float,default=0)
    ap.add_argument('--output',type=Path)
    args=ap.parse_args()
    profile=load_profile(args.profile)
    if args.reader:
        reader(profile,args.side);return
    print(f'Profile: {args.profile.resolve()}',flush=True)
    print('Model mapping signs: '+str(dict(zip(JOINTS,profile['arms'][args.side]['sign']))),flush=True)
    import mujoco
    from .planner import make_model,put_measured
    from ..recording import Recording
    output=args.output or Path('teleop_sessions')/datetime.now().strftime('mirror_%Y%m%d_%H%M%S')
    proc=subprocess.Popen([args.lerobot_python,'-m','webxr_teleop.hardware.mirror','--reader','--side',args.side,'--profile',str(args.profile.resolve())],stdout=subprocess.PIPE,text=True)
    latest=deque(maxlen=1)
    def collect():
        for line in proc.stdout:
            try: latest.append(json.loads(line))
            except ValueError: print(line.rstrip(),file=sys.stderr)
    thread=threading.Thread(target=collect,daemon=True);thread.start()
    recording=None;viewer=None;stop=False
    def request_stop(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,request_stop);signal.signal(signal.SIGINT,request_stop)
    try:
        deadline=time.monotonic()+10
        while not latest and proc.poll() is None and not stop and time.monotonic()<deadline:time.sleep(.02)
        if not latest:raise RuntimeError('No torque-off feedback received; see reader error above')
        model=make_model(profile,(args.side,));data=mujoco.MjData(model)
        # Incorrect offsets can put the arm below the virtual table. Keep the
        # diagnostic visible without changing measured angles or live geometry.
        for name in ('table','floor'):
            model.geom(name).rgba[3]=0
        model.vis.headlight.ambient[:]=[.7,.7,.7]
        put_measured(model,data,profile,{args.side:latest[-1]['q']})
        import numpy as np
        points=np.array([data.xpos[i] for i in range(model.nbody) if model.body(i).name.startswith(args.side+'/')])
        camera=mujoco.MjvCamera();camera.lookat[:]=(points.min(0)+points.max(0))/2
        camera.distance=max(1.,float(np.linalg.norm(points.max(0)-points.min(0)))*2)
        camera.azimuth=0;camera.elevation=-15
        recording=Recording(output,model,30,dict(mode='read_only_torque_off',side=args.side,profile=profile,mapping_verified=False,environment_hidden=True),video_filter="drawtext=text='READ ONLY - TORQUE OFF - MAPPING UNVERIFIED':fontcolor=white:fontsize=16:box=1:boxcolor=black@0.8:x=8:y=8",camera=camera)
        recording.sites=[]
        if not args.headless:
            import mujoco.viewer
            viewer=mujoco.viewer.launch_passive(model,data)
            viewer.cam.lookat[:]=camera.lookat;viewer.cam.distance=camera.distance;viewer.cam.azimuth=camera.azimuth;viewer.cam.elevation=camera.elevation
        print(f'Mirroring {args.side}, torque OFF. No motor writes. MP4: {output.resolve()}/session.mp4',flush=True)
        start=time.monotonic()
        while not stop and (not args.seconds or time.monotonic()-start<args.seconds) and (viewer is None or viewer.is_running()):
            tick=time.monotonic();row=latest[-1]
            if tick-row['t']>.3 or proc.poll() is not None:
                raise RuntimeError('Feedback lost; mirror stopped (no motor commands)')
            put_measured(model,data,profile,{args.side:row['q']})
            recording.write(data,row)
            if viewer: viewer.sync()
            time.sleep(max(0,1/30-(time.monotonic()-tick)))
    finally:
        if viewer:viewer.close()
        if recording:recording.close()
        if proc.poll() is None:proc.terminate()
        proc.wait(timeout=5)
        proc.stdout.close()


if __name__=='__main__':main()
