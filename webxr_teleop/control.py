"""ViperX adaptation of StairVLA's relative WebXR teleoperation workflow.

World frame: +X right, +Y forward, +Z up. Position units are metres throughout.
Uses SO(3) rotation filtering rather than independent Euler-angle filtering.
"""
from dataclasses import dataclass, field
import math
import mujoco
import numpy as np
from .mink_ik import MinkIK
from .planar_limits import PlanarBounds
from scipy.spatial.transform import Rotation

JOINTS = ('waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate')


@dataclass
class Config:
    scale: float = 1.0
    rotation_scale: float = 1.0
    timeout: float = 0.5
    linear_speed: float = 0.35
    angular_speed: float = math.radians(160)
    filter_tau: float = 0.06
    position_deadband: float = 0.002
    rotation_deadband: float = math.radians(1)
    max_jump: float = 0.12
    planar_bounds: PlanarBounds = field(default_factory=PlanarBounds)
    joint_speed: float = math.radians(60)
    ik_damping: float = 0.02
    ik_posture_cost: float = 0.005
    home_speed: float = math.radians(8)


class Teleop:
    def __init__(self, model, data, config=None, side="right"):
        self.model, self.data = model, data
        self.side = side
        prefix = side + "/"
        self.cfg = config or Config()
        self.joints = [model.joint(prefix + name).id for name in JOINTS]
        self.qids = model.jnt_qposadr[self.joints]
        self.aids = [model.actuator(prefix + name).id for name in JOINTS]
        self.fingers = [model.jnt_qposadr[model.joint(prefix + name + '_finger').id] for name in ('left', 'right')]
        self.gripper = model.actuator(prefix + 'gripper').id
        self.bounds = model.jnt_range[self.joints].T.copy()
        self.bounds[0] += 0.02
        self.bounds[1] -= 0.02
        self.site = model.site(prefix + 'gripper').id
        self.scratch = mujoco.MjData(model)
        self.ik = MinkIK(model, data, self.joints, self.site, joint_speed=self.cfg.joint_speed,
                         damping_cost=self.cfg.ik_damping, posture_cost=self.cfg.ik_posture_cost,
                         planar_bounds=self.cfg.planar_bounds)
        self.ik_position_error = self.ik_rotation_error = self.ik_joint_speed = 0.0
        self.ik_solved = True
        self.plane_limited = False
        self.ik_reason = ''
        self.home = data.qpos[self.qids].copy()
        self.base_offset = data.xpos[model.body(prefix + 'base_link').id].copy()
        self.base_offset[2] = 0  # Preserve the single-arm world-height limits.
        self.basis = np.array([[1., 0, 0], [0, 0, -1.], [0, 1., 0]])
        self.calibrated = False
        self.engaged = False
        self.released = False
        self.homing = False
        self.tracking = False
        self.ik_ok = True
        self.clipped = False
        self.message = 'Point controller -Z forward (blue +Z backward), release grip, press left X to align.'
        self.buttons = {}
        self.p, self.r = self.actual_pose()
        self.target_p, self.target_r = self.p.copy(), self.r.copy()
        self.last_hand = None
        self.last_stamp = None
        self.events = []

    def actual_pose(self):
        return self.data.site_xpos[self.site].copy(), self.data.site_xmat[self.site].reshape(3, 3).copy()

    def release(self):
        self.engaged = False
        self.clipped = False
        self.ik_ok = True
        self.target_p, self.target_r = self.actual_pose()
        self.ik_position_error = self.ik_rotation_error = self.ik_joint_speed = 0.0
        self.ik_solved = True
        self.plane_limited = False
        self.ik_reason = ''
        self.last_hand = None
        self.last_stamp = None

    def reset_tracking(self):
        self.release()
        self.calibrated = False
        self.released = False
        self.buttons = {}
        self.message = 'Tracking frame changed: release grip and press left X to align again.'

    def calibrate(self, quaternion):
        # WebXR grip frame: +X right, +Y up, +Z backward. Gravity supplies up.
        forward = Rotation.from_quat(quaternion).apply([0., 0, -1.])
        forward[1] = 0
        if np.linalg.norm(forward) < 0.2:
            self.message = 'Controller -Z is too vertical. Hold blue +Z horizontally backward.'
            return False
        forward /= np.linalg.norm(forward)
        up = np.array([0., 1., 0.])
        right = np.cross(forward, up)
        self.basis = np.stack([right, forward, up])
        self.calibrated = True
        self.release()
        self.message = f'Aligned: right +X, forward +Y, up +Z. Hold {self.side} grip to move.'
        return True

    def start_home(self):
        self.release()
        self.released = False
        self.homing = True
        self.message = 'Returning home; teleop paused.'

    def solve(self, p, r, dt=1/30):
        result = self.ik.advance(self.data, p, r, dt)
        self.ik_solved = result.accepted
        self.ik_reason = result.reason
        self.plane_limited = result.plane_limited
        self.ik_position_error, self.ik_rotation_error = result.position_error, result.rotation_error
        self.ik_joint_speed = result.joint_speed
        self.ik_ok = result.accepted and result.position_error <= 0.005 and result.rotation_error <= math.radians(5)
        if result.accepted:
            self.data.ctrl[self.aids] = self.data.qpos[self.qids]
        return result.accepted

    def step(self, left, right, now, dt, allow_alignment=True):
        self.events = []
        self.plane_limited = False
        dt = float(np.clip(dt, 0, 0.05))
        left = left if left is not None and 0 <= now-left.received_t < self.cfg.timeout else None
        right = right if right is not None and 0 <= now-right.received_t < self.cfg.timeout else None
        self.tracking = (left if self.side == "left" else right) is not None
        states = dict(left_x=bool(left and left.pressed('x')), left_y=bool(left and left.pressed('y')),
                      right_a=bool(right and right.pressed('a')), right_b=bool(right and right.pressed('b')))
        self.events = [key for key, val in states.items() if val and not self.buttons.get(key)]
        self.buttons = states
        if allow_alignment and 'left_x' in self.events and right and not right.squeeze and not self.homing and (self.side == 'right' or (left and not left.squeeze)):
            if self.calibrate(right.quaternion):
                self.events.append('calibrated')
        if 'left_y' in self.events:
            self.start_home()
        if 'right_b' in self.events and self.homing:
            self.homing = False
            self.release()
            self.message = 'Home cancelled; release grip before moving.'
        if self.homing:
            q = self.data.qpos[self.qids]
            speed = min(self.cfg.home_speed, self.cfg.joint_speed)
            q = q + np.clip(self.home - q, -speed*dt, speed*dt)
            proposal = self.data.qpos.copy()
            proposal[self.qids] = q
            fraction = self.ik.planes.safe_fraction(self.data.qpos, proposal)
            q = self.data.qpos[self.qids]+fraction*(q-self.data.qpos[self.qids])
            self.data.qpos[self.qids] = q
            self.data.ctrl[self.aids] = q
            mujoco.mj_forward(self.model, self.data)
            self.release()
            if fraction < 1:
                self.homing = False
                self.plane_limited = True
                self.events.append('home_blocked')
                self.message = 'Homing blocked by a planar boundary; reposition with the controller.'
                return
            if np.max(np.abs(q - self.home)) < 1e-5:
                self.homing = False
                self.events.append('home_reached')
                self.message = 'Home reached. Release grip before moving.'
            return
        right = left if self.side == "left" else right
        if right is None:
            self.release()
            self.released = False
            self.message = f'{self.side.title()} controller unavailable; holding position.'
            return  # Gripper also holds on disconnect.
        lo, hi = self.model.actuator_ctrlrange[self.gripper]
        finger = hi - right.trigger * (hi-lo)
        proposal = self.data.qpos.copy()
        proposal[self.fingers] = finger
        finger_fraction = self.ik.planes.safe_fraction(self.data.qpos, proposal)
        self.data.qpos[self.fingers] += finger_fraction*(finger-self.data.qpos[self.fingers])
        self.data.ctrl[self.gripper] = self.data.qpos[self.fingers[0]]
        self.plane_limited |= finger_fraction < 1
        mujoco.mj_forward(self.model, self.data)
        if not right.squeeze:
            self.release()
            self.released = True
            self.ik_ok = True
            self.plane_limited = finger_fraction < 1
            if self.plane_limited:
                self.message = 'Gripper stopped at a planar boundary.'
            return
        if not self.calibrated or not self.released:
            self.release()
            self.message = 'Release grip and align with left X before moving.'
            return
        if not self.engaged:
            self.ik.set_reference(self.data.qpos)
            self.hand_p = right.position.copy()
            self.hand_r = Rotation.from_quat(right.quaternion).as_matrix()
            self.origin_p, self.origin_r = self.actual_pose()
            self.target_p, self.target_r = self.origin_p.copy(), self.origin_r.copy()
            self.last_hand = right.position.copy()
            self.last_stamp = right.received_t
            self.engaged = True
        if right.received_t != self.last_stamp:
            if np.linalg.norm(right.position - self.last_hand) > self.cfg.max_jump:
                self.release()
                self.released = False
                self.message = 'Tracking jump: release grip to resume.'
                self.events.append('tracking_jump')
                return
            self.last_hand = right.position.copy()
            self.last_stamp = right.received_t
        # Express WebXR deltas in the EEF frame frozen at clutch engagement.
        # IK consumes world poses, so transform back once when constructing its target.
        # Freezing the frame avoids steering a held displacement as the wrist rotates.
        xr_to_eef = self.origin_r.T @ self.basis
        dp_eef = xr_to_eef @ (right.position-self.hand_p)
        raw_p = self.origin_p + self.cfg.scale * self.origin_r @ dp_eef
        bounded = np.clip(raw_p, self.base_offset + [-0.45, -0.10, 0.12], self.base_offset + [0.45, 0.65, 0.75])
        self.clipped = not np.allclose(raw_p, bounded)
        hand_delta = Rotation.from_quat(right.quaternion).as_matrix() @ self.hand_r.T
        dr = Rotation.from_matrix(xr_to_eef @ hand_delta @ xr_to_eef.T)
        raw_r = self.origin_r @ Rotation.from_rotvec(dr.as_rotvec()*self.cfg.rotation_scale).as_matrix()
        dp = bounded - self.target_p
        rv = Rotation.from_matrix(raw_r @ self.target_r.T).as_rotvec()
        pn, rn = np.linalg.norm(dp), np.linalg.norm(rv)
        if pn < self.cfg.position_deadband:
            dp[:] = 0
        if rn < self.cfg.rotation_deadband:
            rv[:] = 0
        alpha = 1 - math.exp(-dt / self.cfg.filter_tau) if self.cfg.filter_tau else 1.0
        dp *= min(alpha, self.cfg.linear_speed*dt/max(pn, 1e-12))
        rv *= min(alpha, self.cfg.angular_speed*dt/max(rn, 1e-12))
        p = self.target_p + dp
        r = Rotation.from_rotvec(rv).as_matrix() @ self.target_r
        accepted = self.solve(p, r, dt)
        self.plane_limited |= finger_fraction < 1
        if accepted:
            self.target_p, self.target_r = p, r
            self.message = ('Planar boundary reached.' if self.plane_limited else
                            'Tracking limited near a singularity, joint limit, or unreachable pose.' if not self.ik_ok else
                            'Workspace limit reached.' if self.clipped else f'Following {self.side} controller.')
        else:
            self.message = 'IK solve failed; holding joint command.'
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def frames_in_vr(self):
        _, r = self.actual_pose()
        return (Rotation.from_matrix(self.basis.T).as_quat().tolist(),
                Rotation.from_matrix(self.basis.T @ r).as_quat().tolist())
