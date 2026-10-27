"""Mink and geometry work run outside the supervisor and motor watchdogs."""
import math
from pathlib import Path
import socket
import time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from ..mink_ik import MinkIK
from ..planar_limits import PlanarBounds
from .core import SIDES, JOINTS, HomeTrajectory, PERIOD, TTL, SettlingWindow, teleop_scales
from .ipc import send, receive
from .motion import motion_settings
from .gripper import aperture_from_trigger, shape_aperture, gripper_speed, gripper_acceleration

ROOT = Path(__file__).resolve().parents[2]


def make_model(profile, sides=SIDES):
    model = mujoco.MjModel.from_xml_path(str(ROOT/'scenes/dual_viperx_scene.xml'))
    for side in SIDES:
        a = profile['arms'][side]
        body = model.body(side+'/base_link').id
        model.body_pos[body] = a['base_position']
        model.body_quat[body] = a['base_quaternion_wxyz']
        for i,n in enumerate(JOINTS):
            j = model.joint(side+'/'+n).id
            lo,hi = model.jnt_range[j]
            # Never widen the source model's mechanical limits.
            model.jnt_range[j] = [max(lo,a['lower_rad'][i]),min(hi,a['upper_rad'][i])]
    table = model.geom('table').id
    model.geom_pos[table,2] = profile['table_top_z']-model.geom_size[table,2]
    for geom in range(model.ngeom):
        name=model.body(model.geom_bodyid[geom]).name or ''
        if '/' in name and name.split('/')[0] not in sides:
            model.geom_rgba[geom,3]=0
    return model


def put_measured(model, data, profile, measured):
    for side in SIDES:
        a = profile['arms'][side]; q = measured.get(side,a['initial_rad']+[a['initial_gripper']])
        for n,v in zip(JOINTS,q):
            data.qpos[model.jnt_qposadr[model.joint(side+'/'+n).id]] = v
        opening = a['finger_closed_m']+q[6]*(a['finger_open_m']-a['finger_closed_m'])
        for finger in ('left','right'):
            data.qpos[model.jnt_qposadr[model.joint(side+'/'+finger+'_finger').id]] = opening
    mujoco.mj_forward(model,data)


class Planner:
    def __init__(self, profile, measured, sides=SIDES):
        self.profile = profile
        self.translation_scale,self.rotation_scale=teleop_scales(profile)
        self.gripper_speed=gripper_speed(profile)
        self.gripper_acceleration=gripper_acceleration(profile)
        self.catch_up = profile.get('teleop_catch_up',False)
        if not isinstance(self.catch_up,bool):raise ValueError('teleop_catch_up must be boolean')
        self.joint_speed,self.joint_acceleration=motion_settings(profile)
        self.sides = sides
        self.model = make_model(profile,sides)
        self.data = mujoco.MjData(self.model)
        put_measured(self.model,self.data,profile,measured)
        self.iks = {}; self.qids = {}; self.sites = {}
        for side in self.sides:
            joints = [self.model.joint(side+'/'+n).id for n in JOINTS]
            self.qids[side] = self.model.jnt_qposadr[joints]
            self.sites[side] = self.model.site(side+'/gripper').id
            self.iks[side] = MinkIK(self.model,self.data,joints,self.sites[side],
                joint_speed=math.radians(self.joint_speed),joint_acceleration=math.radians(self.joint_acceleration),
                planar_bounds=PlanarBounds(**profile['planes']))
        self.epoch = -1
        self.basis = None
        self.grip_generation={}
        self.origin = {}
        self.last = {s:np.array(measured[s]) for s in self.sides}
        self.velocity = {s:np.zeros(7) for s in self.sides}
        self.last_time = None
        self.frozen = {s:True for s in self.sides}
        self.home = None
        self.home_start = None
        self.home_id = None
        self.previous_measured = None
        self.settled_since = None

    def valid_path(self, start, end, *, joint_limits=True):
        for side in self.sides:
            q=end[self.qids[side]]
            ik=self.iks[side]
            if joint_limits and (np.any(q < ik.configuration_limit.lower[self.qids[side]]) or np.any(q > ik.configuration_limit.upper[self.qids[side]])):
                return False
            if ik.planes.safe_fraction(start,end) < 1:
                return False
        return True

    def align(self, quaternion):
        f = Rotation.from_quat(quaternion).apply([0,0,-1]); f[1]=0
        if np.linalg.norm(f) < .2:
            raise ValueError('Point right controller horizontally forward to align')
        f /= np.linalg.norm(f); up=np.array([0.,1.,0.])
        self.basis=np.stack([np.cross(f,up),f,up])

    def step(self, job):
        now=time.monotonic()
        if not 0 <= now-job['t'] <= TTL:
            raise ValueError('Planner input expired')
        measured=job['measured']
        put_measured(self.model,self.data,self.profile,measured)
        start=self.data.qpos.copy()
        measured_eef={s:dict(position=self.data.site_xpos[self.sites[s]].tolist(),
            quaternion_xyzw=Rotation.from_matrix(self.data.site_xmat[self.sites[s]].reshape(3,3)).as_quat().tolist()) for s in self.sides}
        if job['epoch'] != self.epoch:
            self.epoch=job['epoch']; self.origin={}; self.basis=None; self.home=None; self.home_id=None
            self.last={s:np.array(measured[s]) for s in self.sides}
            self.velocity={s:np.zeros(7) for s in self.sides}; self.last_time=None
            self.frozen={s:True for s in self.sides}
        dt=PERIOD if self.last_time is None else now-self.last_time
        # Initialization or scheduling jitter can put consecutive fresh jobs
        # less than 5 ms apart. A short positive interval is not a missed
        # deadline; retain the actual dt for conservative motion shaping.
        if not 0 < dt <= TTL:
            raise ValueError(f'Planner missed control deadline (interval {dt:.6f}s; maximum {TTL:.3f}s)')
        self.last_time=now
        measured_speed=0. if self.previous_measured is None else max(abs(measured[s][j]-self.previous_measured[s][j])/dt for s in self.sides for j in range(6))
        self.previous_measured={s:measured[s][:] for s in self.sides}
        mode=job['mode']; hands=job['hands']; freeze={s:False for s in self.sides}
        restarting=job.get('restart',False)
        if job.get('prepare_restart'):
            end=start.copy()
            for s in self.sides:end[self.qids[s]]=self.profile['arms'][s]['opening_rad']
            if not self.valid_path(start,end):raise ValueError('Restart path violates joint/planar limits')
        if mode=='align':
            self.align(hands['right']['quaternion'])
        targets={s:np.array(measured[s]) for s in self.sides}
        diagnostics={}
        home_done=False
        trajectory_start=False
        if mode in ('home', 'opening', 'sleep'):
            if self.home_id != (mode, job['home_id']):
                trajectory_start=True
                self.home_id=(mode, job['home_id'])
                begin=sum([measured[s][:6] for s in self.sides],[])
                goal=sum([self.profile['arms'][s][{'home':'home_rad', 'opening':'opening_rad', 'sleep':'sleep_rad'}[mode]] for s in self.sides],[])
                self.home=HomeTrajectory(begin,goal,speed=math.radians(10 if mode=='opening' else 5),
                    **({'acceleration':math.radians(30)} if restarting else {}))
                end=start.copy()
                for i,s in enumerate(self.sides):
                    end[self.qids[s]]=goal[6*i:6*i+6]
                if not self.valid_path(start,end,joint_limits=mode!='sleep'):
                    raise ValueError('Whole home path violates joint/planar limits')
                self.home_start=now; self.home_grippers={s:measured[s][6] for s in self.sides}
                self.origin={}; self.settled_since=None
                # Operator-requested restart arrival tolerance; sustained
                # stationary feedback is still required before handoff.
                arrival_deg=(5 if restarting else 2) if mode=='opening' else .5
                self.arrival={s:SettlingWindow(arm_error=math.radians(arrival_deg)) for s in self.sides}
                self.last={s:np.array(measured[s]) for s in self.sides}
                self.velocity={s:np.zeros(7) for s in self.sides}
                dt=PERIOD
            time_scale=3 if mode in ('opening','sleep') and not restarting else 1
            elapsed=now-self.home_start
            t=elapsed*time_scale
            q=self.home.at(t)
            for i,s in enumerate(self.sides):
                targets[s]=np.array(q[i*6:i*6+6]+[self.home_grippers[s]])
            if t >= self.home.duration:
                checks=[self.arrival[s].update(measured[s],targets[s].tolist(),now) for s in self.sides]
                home_done=all(checks)
                # Opening's wall-clock handoff is owned by the supervisor.
                if (mode!='opening' or restarting) and elapsed > self.home.duration/time_scale+3 and not home_done:
                    error=max(abs(x-y) for s in self.sides for x,y in zip(measured[s][:6],targets[s][:6]))
                    raise ValueError(f'{mode.capitalize()} settling timed out; maximum joint error {math.degrees(error):.2f} degrees')
        elif mode=='teleop':
            if self.basis is None:
                raise ValueError('Alignment required')
            for s in self.sides:
                generation=job.get('grip_generation',{}).get(s,0)
                if self.grip_generation.get(s)!=generation:
                    self.origin.pop(s,None);self.grip_generation[s]=generation
                hand=hands[s]; site=self.sites[s]
                if hand is None or ('t' in hand and not 0<=now-hand['t']<=TTL):
                    raise ValueError('Controller input expired at planner')
                targets[s][6]=aperture_from_trigger(hand['trigger'])
                if not hand['grip']:
                    self.origin.pop(s,None); freeze[s]=True
                    continue
                p=np.array(hand['position']); r=Rotation.from_quat(hand['quaternion']).as_matrix()
                if s not in self.origin:
                    self.origin[s]=dict(hand=p,rotation=r,p=self.data.site_xpos[site].copy(),
                        r=self.data.site_xmat[site].reshape(3,3).copy(),last_hand=p,
                        target_p=self.data.site_xpos[site].copy(),
                        target_r=self.data.site_xmat[site].reshape(3,3).copy())
                    # A new grip also resets joint integration to current feedback.
                    self.last[s][:6]=measured[s][:6];self.velocity[s][:6]=0
                    self.iks[s].set_reference(self.data.qpos)
                o=self.origin[s]
                if np.linalg.norm(p-o['last_hand']) > .12:
                    raise ValueError('Controller tracking jump')
                o['last_hand']=p
                desired_p=o['p']+self.translation_scale*(self.basis@(p-o['hand']))
                delta_r=self.basis@r@o['rotation'].T@self.basis.T
                desired_r=Rotation.from_rotvec(Rotation.from_matrix(delta_r).as_rotvec()*self.rotation_scale).as_matrix()@o['r']
                if self.catch_up:
                    dp=desired_p-o['target_p']; rv=Rotation.from_matrix(desired_r@o['target_r'].T).as_rotvec()
                    alpha=1-math.exp(-dt/.06)
                    o['target_p']+=dp*min(alpha,.05*dt/max(np.linalg.norm(dp),1e-12))
                    o['target_r']=Rotation.from_rotvec(rv*min(alpha,math.radians(30)*dt/max(np.linalg.norm(rv),1e-12))).as_matrix()@o['target_r']
                else:
                    o['target_p']=desired_p.copy()
                    o['target_r']=desired_r.copy()
                # Integrate IK from the last commanded configuration. Re-seeding
                # every step from feedback turns persistent motor following error
                # (e.g. gravity sag) into cumulative command drift. Feedback still
                # anchors clutch engagement and validates the final target path;
                # the archived bounded-lead option remains disabled by default.
                self.data.qpos[self.qids[s]]=self.last[s][:6]
                mujoco.mj_forward(self.model,self.data)
                result=self.iks[s].advance(self.data,o['target_p'],o['target_r'],dt)
                if not result.accepted:
                    raise ValueError('IK rejected target: '+result.reason)
                diagnostics[s]=dict(desired_position=desired_p.tolist(),filtered_position=o['target_p'].tolist(),
                    ik_position_error_m=result.position_error,ik_rotation_error_rad=result.rotation_error,
                    plane_limited=result.plane_limited,alignment_basis=self.basis.tolist(),
                    desired_quaternion_xyzw=Rotation.from_matrix(desired_r).as_quat().tolist(),
                    measured_eef_from_encoders=measured_eef[s],
                    controller_age_upper_bound_ms=(1000*(now-hand['t']) if 't' in hand else None))
                targets[s][:6]=self.data.qpos[self.qids[s]]
        else:
            self.origin={}
            freeze={s:True for s in self.sides}
        # Arm clutch and absolute gripper aperture have independent authority.
        for s in self.sides:
            if mode=='teleop':
                if freeze[s]:
                    targets[s][:6]=self.last[s][:6] if self.frozen[s] else measured[s][:6]
                    self.velocity[s][:6]=0
                else:
                    self.velocity[s][:6]=(targets[s][:6]-self.last[s][:6])/dt
                targets[s][6],self.velocity[s][6]=shape_aperture(
                    self.last[s][6],self.velocity[s][6],targets[s][6],dt,speed_limit=self.gripper_speed,acceleration_limit=self.gripper_acceleration)
                diagnostics.setdefault(s,{})['desired_aperture']=aperture_from_trigger(hands[s]['trigger'])
            elif freeze[s]:
                targets[s]=self.last[s].copy() if self.frozen[s] else np.array(measured[s])
                self.velocity[s]=np.zeros(7)
            if self.catch_up and mode=='teleop' and not freeze[s]:
                # Pause command advancement at a bounded lead rather than
                # disarming or continually moving the goal away from the motor.
                previous=self.last[s]
                delta=targets[s]-previous
                alpha=1.
                for j in range(6):
                    error=previous[j]-measured[s][j]
                    step=delta[j]
                    if abs(error)>math.radians(2):
                        alpha=0.;break
                    if step>0:alpha=min(alpha,(math.radians(2)-error)/step)
                    elif step<0:alpha=min(alpha,(-math.radians(2)-error)/step)
                alpha=max(0.,min(1.,alpha))
                targets[s][:6]=previous[:6]+alpha*delta[:6]
                if alpha<1:self.velocity[s][:6]=0
                diagnostics.setdefault(s,{})['catching_up']=bool(alpha<1)
            if mode in ('home','opening','sleep'):
                self.velocity[s]=(targets[s]-self.last[s])/dt
            self.frozen[s]=freeze[s]
            self.last[s]=targets[s].copy()
        put_measured(self.model,self.data,self.profile,{s:targets[s].tolist() for s in self.sides})
        if not self.valid_path(start,self.data.qpos,joint_limits=mode not in ('opening','sleep','hold')):
            raise ValueError('Finite target path violates joint/planar limits')
        for s in self.sides:
            diagnostics.setdefault(s,{}).update(requested_joint_velocity_rad_s=self.velocity[s][:6].tolist(),
                measured_eef_from_encoders=measured_eef[s])
        return dict(gripper_active=mode=='teleop',grip_generation=job.get('grip_generation',{}),
                    controller_t={s:(hands[s].get('t') if hands.get(s) else None) for s in self.sides},
                    diagnostics=diagnostics,seq=job['seq'],q={s:targets[s].tolist() for s in self.sides},freeze=freeze,issued=now,
                    input_t=job['t'],epoch=job['epoch'],mode=mode,home_done=home_done,
                    arrival_error_deg=(math.degrees(max(abs(measured[s][j]-self.home.goal[i*6+j]) for i,s in enumerate(self.sides) for j in range(6))) if mode in ('home','opening','sleep') else None),trajectory_start=trajectory_start,trajectory_id=job.get('home_id'))


def run(sock, profile, sides=SIDES):
    planner=None
    try:
        while True:
            jobs=receive(sock)
            if not jobs:
                time.sleep(.002); continue
            job=jobs[-1]
            if job.get('op')=='shutdown':
                break
            if planner is None:
                planner=Planner(profile,job['measured'],sides)
            if time.monotonic()-job['t'] > TTL:
                send(sock,dict(type='expired',epoch=job['epoch'],mode=job['mode'],seq=job['seq']))
                continue
            started=time.monotonic()
            target=dict(type='target',**planner.step(job))
            target['planner_duration_ms']=1000*(time.monotonic()-started)
            target['planner_queue_age_ms']=1000*(started-job['t'])
            target['completed_t']=time.monotonic()
            try:
                send(sock,target)
            except TypeError as exc:
                # No target was sent. Keep the planner alive so an explicit
                # sleep request can replace teleop in a fresh generation.
                send(sock,dict(type='fault',reason=str(exc),code='serialization',
                               recoverable=True,epoch=job['epoch']))
    except BaseException as exc:
        send(sock,dict(type='fault',reason=str(exc),code='ik' if str(exc).startswith('IK rejected target:') else 'planner',
                       recoverable=str(exc).startswith('IK rejected target:')))
    finally:
        sock.close()
