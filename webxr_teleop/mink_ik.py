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


class MinkIK:
    def __init__(self, model, data, joint_ids, site, *, joint_speed=math.radians(60),
                 damping_cost=0.02, posture_cost=0.005, planar_bounds=None):
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
        self.tasks = [self.frame, self.posture, mink.DampingTask(model, cost=damping_cost)]
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
        self.limits = [self.configuration_limit, mink.VelocityLimit(model, velocity_limits), self.planes]

    def set_reference(self, qpos):
        """A small posture preference towards the pose at clutch engagement."""
        self.posture.set_target(qpos.copy())

    def advance(self, data, p, r, dt):
        if not np.isfinite(dt) or dt <= 0 or not np.all(np.isfinite(p)) or not np.all(np.isfinite(r)):
            return IKResult(False, 0., 0., reason='Invalid target or timestep')
        dt = min(dt, 0.05)
        if np.min(self.planes.distances(data)) < -1e-8:
            return IKResult(False, 0., 0., reason='Arm is outside planar bounds', plane_limited=True)
        self.configuration.update(data.qpos)
        self.frame.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(r), p))
        try:
            velocity = mink.solve_ik(self.configuration, self.tasks, dt, solver='daqp',
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
        data.qpos[self.qids] += fraction*(candidate-data.qpos[self.qids])
        speed *= fraction
        mujoco.mj_forward(self.model, data)
        actual_p = data.site_xpos[self.site]
        actual_r = data.site_xmat[self.site].reshape(3, 3)
        return IKResult(True, float(np.linalg.norm(p-actual_p)),
                        float(np.linalg.norm(Rotation.from_matrix(r@actual_r.T).as_rotvec())), speed,
                        plane_limited=bool(fraction < 1 or np.min(self.planes.distances(data)) < .001))
