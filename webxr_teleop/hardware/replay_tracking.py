"""Offline tracking comparison. Never imports a bus or opens a serial device."""
import argparse
import copy
import importlib.machinery
import importlib.util
import json
import math
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from . import planner
from .core import load_profile, SIDES
from .run import DEFAULT_PROFILE
from .motion import motor_profile_counts, VELOCITY_UNIT_DEG_S, ACCELERATION_UNIT_DEG_S2

ROOT=Path(__file__).resolve().parents[2]


def baseline_planner():
    path=ROOT/'tests/fixtures/tracking_baseline_planner.py.snapshot'
    loader=importlib.machinery.SourceFileLoader('webxr_teleop.hardware._tracking_baseline',str(path))
    spec=importlib.util.spec_from_loader(loader.name,loader)
    module=importlib.util.module_from_spec(spec);loader.exec_module(module)
    return module.Planner


def synthetic_frames():
    frames=[]
    for i in range(450):
        t=i*.02
        hands={s:dict(position=[0.,1.2,-.4],quaternion=[0.,0.,0.,1.],grip=False,trigger=0.) for s in SIDES}
        # Two independent strokes, each translating 6 cm and rotating 15 degrees.
        gripping=(.2<=t<3.2 or 4.2<=t<7.2)
        second=t>=3.2
        u=max(0.,min(1.,(t-(4.2 if second else .2))/.6))
        h=hands['left'];h['grip']=gripping
        h['position']=[.3 if second else 0.,1.2+.06*u,-.4]
        h['quaternion']=Rotation.from_euler('y',(.5 if second else 0.)+math.radians(15)*u).as_quat().tolist()
        frames.append(dict(t=t,mode='align' if i==0 else 'teleop',hands=hands))
    return frames


def recorded_frames(path):
    rows=[];profile=None
    for line in Path(path).open():
        try:r=json.loads(line)
        except ValueError:continue
        if r.get('event')=='startup':profile=r['profile']
        if r.get('event')=='tick':rows.append(r)
    frames=[];q=None;epoch=None;start=None
    # First uninterrupted aligned epoch only; never replay a logged fault as motion.
    for r in rows:
        cmd=r.get('command')
        if not cmd:continue
        if q is None:
            if cmd['mode']!='align':continue
            q=r['measured'];epoch=cmd['epoch'];start=r['t']
        if cmd['epoch']!=epoch or r['state']!='ARMED':break
        frames.append(dict(t=r['t']-start,mode=cmd['mode'],hands=r['hands']))
    if not frames:raise ValueError('No aligned teleop segment in recording')
    return profile,q,frames


def compare_case(cls,profile,q,frames,plant='ideal',legacy=False):
    p=copy.deepcopy(profile);q=copy.deepcopy(q);sides=tuple(q)
    clock=[0.];positions=[];rotations=[];cmd_vel=[];cmd_acc=[];release_errors=[];fail=None;active=False;generation={s:0 for s in sides}
    previous_cmd=None;previous_velocity=np.zeros(6);motor_v={s:np.zeros(6) for s in sides}
    with patch.object(planner.time,'monotonic',lambda:clock[0]):
        instance=cls(p,q,sides)
        prev_t=-.02
        for i,f in enumerate(frames):
            clock[0]=f['t'];dt=clock[0]-prev_t;prev_t=clock[0]
            if dt<=0:continue
            hands=copy.deepcopy(f['hands'])
            for h in hands.values():
                if h:h['t']=clock[0]
            gripping=bool(hands['left'] and hands['left']['grip'])
            if gripping!=active:generation['left']+=1
            try:
                result=instance.step(dict(t=clock[0],seq=i,epoch=1,mode=f['mode'],measured=q,hands=hands,grip_generation=generation))
            except ValueError as exc:
                fail=dict(frame=i,reason=str(exc));break
            command=np.array(result['q']['left'][:6])
            if gripping and active and previous_cmd is not None:
                velocity=(command-previous_cmd)/dt
                cmd_vel.append(np.max(abs(velocity)))
                cmd_acc.append(np.max(abs(velocity-previous_velocity))/dt)
                previous_velocity=velocity
            else:previous_velocity=np.zeros(6)
            if not gripping and active:release_errors.append(float(np.max(abs(command-np.array(q['left'][:6])))))
            if gripping and f['mode']=='teleop':
                o=instance.origin['left']
                desired=o['p']+instance.translation_scale*(instance.basis@(np.array(hands['left']['position'])-o['hand']))
                dr=instance.basis@Rotation.from_quat(hands['left']['quaternion']).as_matrix()@o['rotation'].T@instance.basis.T
                desired_r=Rotation.from_rotvec(Rotation.from_matrix(dr).as_rotvec()*instance.rotation_scale).as_matrix()@o['r']
                planner.put_measured(instance.model,instance.data,p,q)
                site=instance.sites['left']
                positions.append(float(np.linalg.norm(desired-instance.data.site_xpos[site])))
                rotations.append(float(np.linalg.norm(Rotation.from_matrix(desired_r@instance.data.site_xmat[site].reshape(3,3).T).as_rotvec())))
            for s in sides:
                target=np.array(result['q'][s]);actual=np.array(q[s])
                if plant=='ideal':actual=target
                else:
                    # Explicit approximation: first-order servo plus motor speed/acceleration bounds.
                    # NOT a Dynamixel firmware, load, friction, or gravity simulation.
                    vc,ac=(14,4) if legacy else motor_profile_counts(p.get('teleop_joint_speed_deg_s',18),p.get('teleop_joint_acceleration_deg_s2',60))
                    wanted=np.clip((target[:6]-actual[:6])/.06,-math.radians(vc*VELOCITY_UNIT_DEG_S),math.radians(vc*VELOCITY_UNIT_DEG_S))
                    motor_v[s]+=np.clip(wanted-motor_v[s],-math.radians(ac*ACCELERATION_UNIT_DEG_S2)*dt,math.radians(ac*ACCELERATION_UNIT_DEG_S2)*dt)
                    actual[:6]+=motor_v[s]*dt;actual[6]=target[6]
                q[s]=actual.tolist()
            active=gripping;previous_cmd=command
    def rms(values):return float(np.sqrt(np.mean(np.square(values)))) if values else None
    return dict(frames_completed=i+1,failure=fail,position_rmse_mm=1000*rms(positions) if positions else None,
                rotation_rmse_deg=math.degrees(rms(rotations)) if rotations else None,
                peak_command_velocity_deg_s=math.degrees(max(cmd_vel,default=0)),
                peak_command_acceleration_deg_s2=math.degrees(max(cmd_acc,default=0)),
                release_command_error_rad=max(release_errors,default=0),plant=plant)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--session',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if args.session:p,q,frames=recorded_frames(args.session)
    else:
        p=load_profile(DEFAULT_PROFILE);q={'left':p['arms']['left']['initial_rad']+[p['arms']['left']['initial_gripper']]};frames=synthetic_frames()
    before=baseline_planner();results={}
    for plant in ('ideal','estimated_servo'):
        for name,cls,speed,acc in [('before',before,9,24),('coordinated_same_limits',planner.Planner,9,24),('after',planner.Planner,18,60)]:
            profile=copy.deepcopy(p);profile.update(teleop_joint_speed_deg_s=speed,teleop_joint_acceleration_deg_s2=acc)
            results[plant+'/'+name]=compare_case(cls,profile,q,frames,plant,legacy=name=='before')
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(dict(source=str(args.session) if args.session else 'synthetic two-stroke translation/rotation',results=results),indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
