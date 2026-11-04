"""Per-arm differential IK; only the selected arm's six hinge joints can move."""
from dataclasses import dataclass
import math

import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from .planar_limits import PlanarLimit


@dataclass
class IKResult:
    accepted: bool
    position_error: float
    rotation_error: float
    joint_speed: float = 0.0
    reason: str = ''
    plane_limited: bool = False


class JointMotionLimit(mink.Limit):
    """Joint speed and acceleration in the same QP; other DOFs remain locked."""
    def __init__(self,model,dofs,speed,acceleration):
        self.dofs=dofs
        self.speed=speed
        self.acceleration=acceleration
        self.previous=np.zeros(len(dofs))
        self.projection=np.eye(model.nv)

    def compute_qp_inequalities(self,configuration,dt):
        lower=np.zeros(configuration.model.nv);upper=lower.copy()
        lower[self.dofs]=np.maximum(-self.speed,self.previous-self.acceleration*dt)
        upper[self.dofs]=np.minimum(self.speed,self.previous+self.acceleration*dt)
        return mink.Constraint(G=np.vstack((self.projection,-self.projection)),
                               h=np.concatenate((upper*dt,-lower*dt)))


class MinkIK:
    def __init__(self, model, data, joint_ids, site, *, joint_speed=math.radians(60),
                 damping_cost=0.02, posture_cost=0.005, planar_bounds=None, joint_acceleration=None,
                 preferred_posture=None, preferred_posture_cost=None, joint_damping_cost=None):
        if not np.isfinite(joint_speed) or joint_speed <= 0:
            raise ValueError('joint_speed must be finite and positive')
        if not all(np.isfinite(x) and x >= 0 for x in (damping_cost, posture_cost)):
            raise ValueError('IK costs must be finite and nonnegative')
        self.model = model
        self.qids = model.jnt_qposadr[joint_ids]
        self.dofs = model.jnt_dofadr[joint_ids]
        self.site = site
        self.joint_speed = joint_speed
        self.configuration = mink.Configuration(model, q=data.qpos.copy())
        self.frame = mink.FrameTask(model.site(site).name, 'site', position_cost=1.0,
                                    orientation_cost=0.25, lm_damping=1.0)
        cost = np.zeros(model.nv)
        cost[self.dofs] = posture_cost
        self.posture = mink.PostureTask(model, cost=cost)
        self.posture.set_target(data.qpos.copy())
        self.tasks = [self.frame, self.posture]
        def arm_vector(value, name):
            array=np.asarray(value,dtype=float)
            if array.shape!=(len(self.dofs),) or not np.all(np.isfinite(array)):
                raise ValueError(f'{name} must contain one finite value per arm joint')
            return array.copy()
        self.preferred_posture=None
        if preferred_posture is not None:
            reference=arm_vector(preferred_posture,'preferred_posture')
            weights=arm_vector(preferred_posture_cost,'preferred_posture_cost')
            if np.any(weights<0):raise ValueError('preferred_posture_cost must be nonnegative')
            bounds=model.jnt_range[joint_ids]
            if np.any(reference<bounds[:,0]) or np.any(reference>bounds[:,1]):
                raise ValueError('preferred_posture must be within joint limits')
            cost=np.zeros(model.nv);cost[self.dofs]=weights
            target=data.qpos.copy();target[self.qids]=reference
            self.preferred_posture=mink.PostureTask(model,cost=cost)
            self.preferred_posture.set_target(target)
            self.tasks.append(self.preferred_posture)
        elif preferred_posture_cost is not None:
            raise ValueError('preferred_posture_cost requires preferred_posture')
        damping=np.zeros(model.nv)
        damping[self.dofs]=(damping_cost if joint_damping_cost is None else
                            arm_vector(joint_damping_cost,'joint_damping_cost'))
        if np.any(damping<0):raise ValueError('joint_damping_cost must be nonnegative')
        self.damping=mink.DampingTask(model,cost=damping)
        self.tasks.append(self.damping)
        # Keep the original 0.02 rad arm margin without shrinking the much smaller
        # finger ranges. Other arms and fingers are locked inside the QP as well
        # as excluded from the eventual write-back.
        self.configuration_limit = mink.ConfigurationLimit(model)
        self.configuration_limit.lower[self.qids] += 0.02
        self.configuration_limit.upper[self.qids] -= 0.02
        names = {model.joint(j).name for j in joint_ids}
        velocity_limits = {model.joint(j).name: joint_speed if model.joint(j).name in names else 0.
                           for j in range(model.njnt)}
        prefix = model.site(site).name.split('/')[0]
        self.planes = PlanarLimit(model, data, prefix+'/base_link', planar_bounds)
        if np.min(self.planes.distances(data)) < -1e-8:
            raise ValueError('Initial arm configuration violates the configured planar bounds')
        if joint_acceleration is not None and (not np.isfinite(joint_acceleration) or joint_acceleration<=0):
            raise ValueError('joint_acceleration must be finite and positive')
        self.motion_limit=(JointMotionLimit(model,self.dofs,joint_speed,joint_acceleration)
                           if joint_acceleration is not None else None)
        self.limits = [self.configuration_limit,
                       self.motion_limit or mink.VelocityLimit(model, velocity_limits), self.planes]

    def set_reference(self, qpos):
        """A small posture preference towards the pose at clutch engagement."""
        self.posture.set_target(qpos.copy())
        if self.motion_limit is not None:self.motion_limit.previous[:]=0

    def advance(self, data, p, r, dt, *, stopping=False):
        if not np.isfinite(dt) or dt <= 0 or not np.all(np.isfinite(p)) or not np.all(np.isfinite(r)):
            return IKResult(False, 0., 0., reason='Invalid target or timestep')
        dt = min(dt, 0.05)
        if np.min(self.planes.distances(data)) < -1e-8:
            return IKResult(False, 0., 0., reason='Arm is outside planar bounds', plane_limited=True)
        self.configuration.update(data.qpos)
        self.frame.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(r), p))
        if self.motion_limit is not None:
            # Unit task gain asks to remove the whole error in one control
            # tick. With acceleration constraints that causes overshoot and
            # repeated reversals even around a nearly stationary target.
            # Express convergence in seconds, independent of planner cadence;
            # scale posture tasks too so their relative preference is unchanged.
            gain = min(1.0, dt / 0.10)
            for task in (self.frame, self.posture, self.preferred_posture):
                if task is not None:
                    task.gain = gain
        try:
            # Braking uses the same joint acceleration, speed and geometry
            # constraints, but removes pose/posture errors that would drive
            # continued motion toward a stationary controller's old target.
            tasks = [self.damping] if stopping else self.tasks
            velocity = mink.solve_ik(self.configuration, tasks, dt, solver='daqp',
                                     damping=1e-8, limits=self.limits, safety_break=True, primal_tol=1e-10)
        except (mink.NoSolutionFound, mink.NotWithinConfigurationLimits) as exc:
            return IKResult(False, 0., 0., reason=str(exc))
        if not np.all(np.isfinite(velocity)):
            return IKResult(False, 0., 0., reason='Non-finite IK velocity')
        speed = float(np.max(np.abs(velocity[self.dofs])))
        if speed > self.joint_speed + 1e-6:
            return IKResult(False, 0., 0., reason='IK velocity limit violation')
        self.configuration.integrate_inplace(velocity, dt)
        candidate = self.configuration.q[self.qids]
        if (np.any(candidate < self.configuration_limit.lower[self.qids] - 1e-7)
                or np.any(candidate > self.configuration_limit.upper[self.qids] + 1e-7)):
            return IKResult(False, 0., 0., reason='IK joint limit violation')
        fraction = self.planes.safe_fraction(data.qpos, self.configuration.q)
        if self.motion_limit is not None and fraction<1-1e-8:
            return IKResult(False,0.,0.,reason='Finite path requires geometry stop',plane_limited=True)
        previous_q=data.qpos[self.qids].copy()
        data.qpos[self.qids] += fraction*(candidate-data.qpos[self.qids])
        if self.motion_limit is not None:
            self.motion_limit.previous=(data.qpos[self.qids]-previous_q)/dt
        speed *= fraction
        mujoco.mj_forward(self.model, data)
        actual_p = data.site_xpos[self.site]
        actual_r = data.site_xmat[self.site].reshape(3, 3)
        return IKResult(True, float(np.linalg.norm(p-actual_p)),
                        float(np.linalg.norm(Rotation.from_matrix(r@actual_r.T).as_rotvec())), speed,
                        plane_limited=bool(fraction < 1 or np.min(self.planes.distances(data)) < .001))
