"""Guided manual, torque-off direction checks; never writes motor registers."""
import argparse
from collections import deque
from copy import deepcopy
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from .core import JOINTS, MOTORS, load_profile


def assess(before, after, index, expected, arm):
    """Infer model-to-calibrated sign from a user-confirmed model direction."""
    names=JOINTS+('gripper',)
    name=names[index]; motor=MOTORS.index(name)
    delta=after['calibrated'][motor]-before['calibrated'][motor]
    threshold=5 if index==6 else 3
    if not threshold<=abs(delta)<=45:
        raise ValueError(f'Move {name} by {threshold}-45 degrees (gripper: percentage points); measured {delta:.2f}')
    others={n:after['calibrated'][MOTORS.index(n)]-before['calibrated'][MOTORS.index(n)] for n in JOINTS if n!=name}
    sign=1 if delta*expected>0 else -1
    raw_delta=[b-a for a,b in zip(before['raw'],after['raw'])]
    shadows={}
    if name in ('shoulder','elbow'):
        shadow=MOTORS.index(name+'_shadow')
        sd=after['calibrated'][shadow]-before['calibrated'][shadow]
        if abs(sd-delta)>2:raise ValueError('Primary/shadow calibrated changes disagree; inspect calibration')
        shadows[name+'_shadow']=dict(calibrated_delta=sd,raw_delta=raw_delta[shadow])
    result=dict(joint=name,calibrated_delta=delta,raw_delta=raw_delta[motor],sign=sign,shadows=shadows,expected_model_delta=expected,other_joint_deltas=others)
    if index<6:
        q0=(before['calibrated'][motor]-arm['offset_deg'][index])/arm['sign'][index]
        result['offset_deg']=before['calibrated'][motor]-sign*q0
    else:
        result['opening_direction_ok']=sign==1
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--side',choices=('left','right'),default='left')
    ap.add_argument('--profile',type=Path,default=Path(__file__).with_name('rig_mirror_candidate.json'))
    ap.add_argument('--lerobot-python',default='/home/King/miniconda3/envs/lerobot/bin/python')
    ap.add_argument('--output',type=Path)
    ap.add_argument('--resume',type=Path,help='Previous session containing results.json and direction_candidate.json')
    ap.add_argument('--reverse-joint',choices=JOINTS+('gripper',),action='append',default=[],help='Reverse the default reference direction for this joint')
    args=ap.parse_args();profile=load_profile(args.profile);candidate=deepcopy(profile)
    resumed=[]
    if args.resume:
        candidate=load_profile(args.resume/'direction_candidate.json')
        profile=deepcopy(candidate)
        resumed=json.loads((args.resume/'results.json').read_text())['results']
        if [r['joint'] for r in resumed]!=list((JOINTS+('gripper',))[:len(resumed)]):
            raise ValueError('Resume results must be an ordered prefix of the joint sequence')
        if len(resumed)>=7:raise ValueError('Previous sequence already complete')
    out=args.output or Path('teleop_sessions')/datetime.now().strftime('directions_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    import numpy as np
    import mujoco
    import mujoco.viewer
    from .planner import make_model,put_measured
    from ..recording import Recording
    reader_profile=args.resume/'direction_candidate.json' if args.resume else args.profile
    proc=subprocess.Popen([args.lerobot_python,'-m','webxr_teleop.hardware.mirror','--reader','--side',args.side,'--profile',str(reader_profile.resolve())],stdout=subprocess.PIPE,text=True)
    latest=deque(maxlen=1);answers=queue.Queue();results=resumed;stop=False;viewer=None;recording=None
    def read():
        for line in proc.stdout:
            try:latest.append(json.loads(line))
            except ValueError:print(line.rstrip(),flush=True)
    def keyboard():
        while True:
            try:answers.put(input())
            except EOFError:return
    def halt(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,halt);signal.signal(signal.SIGINT,halt)
    threading.Thread(target=read,daemon=True).start();threading.Thread(target=keyboard,daemon=True).start()
    def save():
        (out/'results.json').write_text(json.dumps(dict(complete=len(results)==7,results=results,torque_off=True,motor_register_writes=False),indent=2)+'\n')
        candidate['arms'][args.side]['verified']=False
        candidate['arms'][args.side]['verification_note']='Manual direction checks only. Offsets preserve the provisional reference pose; absolute mapping and watchdog remain unverified.'
        candidate['provenance']['mapping']='Partial manual direction-check candidate; see '+str(out.resolve()/'results.json')
        (out/'direction_candidate.json').write_text(json.dumps(candidate,indent=2)+'\n')
    try:
        deadline=time.monotonic()+10
        while not latest and proc.poll() is None and time.monotonic()<deadline and not stop:time.sleep(.02)
        if not latest:raise RuntimeError('No torque-off feedback')
        model=make_model(profile,(args.side,));data=mujoco.MjData(model)
        for n in ('floor','table'):model.geom(n).rgba[3]=0
        model.vis.headlight.ambient[:]=[.7]*3
        viewer=mujoco.viewer.launch_passive(model,data)
        viewer.cam.lookat[:]=[0,0,.2];viewer.cam.distance=1.2;viewer.cam.azimuth=0;viewer.cam.elevation=-10
        overlay=out/'overlay.txt';overlay.write_text('TORQUE OFF - MANUAL DIRECTION CHECK')
        recording=Recording(out/'video',model,30,dict(mode='manual_direction_reference',side=args.side,profile=profile,note='Video shows model reference demonstration, not commanded physical motion.'),video_filter=f'drawtext=textfile={overlay.resolve()}:reload=1:fontcolor=white:fontsize=15:box=1:boxcolor=black@0.8:x=8:y=8',camera=viewer.cam)
        recording.sites=[]
        print('\nTORQUE OFF. No motor writes. Keep the physical arm supported.\nThe viewer will demonstrate ONE joint. Copy its OUTWARD motion by hand.\nDo not follow the return/reset animation. Other joints may move; match the tested joint rotation, not the gripper path.\nEnter: begin / finish movement / accept result. R: retry. Q: quit.\n',flush=True)
        print('MP4:',out.resolve()/'video/session.mp4',flush=True)
        index=len(results);phase='ready';baseline=None;reference=None;result=None;demo_start=0;expected=10
        reversed_joints=set(args.reverse_joint)
        print('F: reverse the current test direction and recapture its starting pose.',flush=True)
        def prompt():print(f'\n{index+1}/7 { (JOINTS+("gripper",))[index] }: Enter to capture starting pose and show direction.',flush=True)
        prompt()
        while not stop and viewer.is_running() and index<7:
            tick=time.monotonic();row=latest[-1]
            if tick-row['t']>.3 or proc.poll() is not None:raise RuntimeError('Feedback lost; no result accepted')
            if not answers.empty():
                answer=answers.get().strip().lower()
                if answer=='q':break
                if answer=='f':
                    name=(JOINTS+('gripper',))[index]
                    reversed_joints.symmetric_difference_update({name})
                    phase='ready';result=None;reference=None
                    print('Direction reversed. Previous movement discarded; Enter captures a fresh baseline.',flush=True);prompt()
                elif answer=='r':phase='ready';result=None;reference=None;prompt()
                elif answer=='':
                    if phase=='ready':
                        baseline=deepcopy(row);reference=row['q'][:]
                        # Reconstruct with accepted candidate signs for following tests.
                        arm=candidate['arms'][args.side]
                        reference[:6]=[math.radians((row['calibrated'][MOTORS.index(n)]-arm['offset_deg'][i])/arm['sign'][i]) for i,n in enumerate(JOINTS)]
                        expected=(10 if reference[index]+math.radians(10)<=arm['upper_rad'][index] else -10) if index<6 else 15
                        if (JOINTS+('gripper',))[index] in reversed_joints:expected=-expected
                        phase='move';demo_start=tick
                        print(f'Copy the shown {expected:+g} '+('degree' if index<6 else 'percentage-point opening')+' direction. Move gently; hold the new pose and press Enter.',flush=True)
                    elif phase=='move':
                        try:result=assess(baseline,row,index,expected,candidate['arms'][args.side])
                        except ValueError as exc:print(str(exc)+'; R to retry.',flush=True);phase='retry'
                        else:
                            print(json.dumps(result,indent=2),flush=True)
                            print('If you moved in the demonstrated direction: Enter to accept; R to repeat.',flush=True);phase='review'
                    elif phase=='review':
                        results.append(result)
                        if index<6:
                            candidate['arms'][args.side]['sign'][index]=result['sign'];candidate['arms'][args.side]['offset_deg'][index]=result['offset_deg']
                        elif not result['opening_direction_ok']:print('Gripper direction disagrees: calibration requires separate correction; no automatic inversion.',flush=True)
                        save();index+=1;phase='ready';reference=None
                        if index<7:prompt()
            displayed=row['q'][:] if reference is None else reference[:]
            if reference is not None and phase in ('move','review','retry'):
                u=min(1.,((tick-demo_start)%5)/2)
                displayed[index]+=expected*u*(math.pi/180 if index<6 else .01)
            put_measured(model,data,profile,{args.side:displayed})
            label=(JOINTS+('gripper',))[index] if index<7 else 'complete'
            overlay.write_text(f'TORQUE OFF - REFERENCE DEMO ONLY\n{index+1}/7 {label} | {phase} | use terminal Enter / R / Q')
            recording.write(data,dict(feedback=row,phase=phase,joint=label,reference_q=displayed))
            viewer.sync();time.sleep(max(0,1/30-(time.monotonic()-tick)))
        save();print('Saved diagnostic candidate and results:',out.resolve(),flush=True)
    finally:
        if viewer:viewer.close()
        if recording:recording.close()
        if proc.poll() is None:proc.terminate()
        proc.wait(timeout=5);proc.stdout.close()

if __name__=='__main__':main()
