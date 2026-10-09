"""Quest ROS 2 input -> dual-arm ALOHA 2 MuJoCo simulation.

This program subscribes only to /quest/* and publishes only /aloha_sim/*.
It has no connection to the physical ALOHA drivers.
"""

import argparse
import json
import math
from pathlib import Path
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


MODEL_PATH = Path(__file__).parent / "mujoco_menagerie/aloha/scene.xml"
SINGLE_MODEL_PATH = Path(__file__).parent / "scenes/single_viperx_scene.xml"
FRAME_ID = "aloha_sim_world"
QUEST_ROTATION_TO_WORLD = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
)
# The reader's reported vertical translation moves opposite to the operator's
# observed up/down motion. Keep the horizontal mapping, invert only translation Z.
QUEST_TRANSLATION_TO_WORLD = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]]
)
ARM_JOINTS = ("waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate")
TRACKING_TIMEOUT = 0.35
GRIP_THRESHOLD = 0.6
MAX_REFERENCE_DISPLACEMENT = 0.30  # m from clutch engagement
MAX_FRAME_JUMP = 0.12  # m per received pose
POSITION_GAIN = 1.0
ROTATION_GAIN = 0.6


def load_calibration(path):
    """Apply a saved proper frame rotation to both translation and orientation."""
    global QUEST_TRANSLATION_TO_WORLD, QUEST_ROTATION_TO_WORLD
    payload = json.loads(Path(path).read_text())
    rotation = np.asarray(payload['quest_to_world_rotation'], dtype=float)
    if (rotation.shape != (3, 3) or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)):
        raise ValueError('Calibration must contain a proper orthonormal 3x3 rotation')
    QUEST_TRANSLATION_TO_WORLD = rotation.copy()
    QUEST_ROTATION_TO_WORLD = rotation.copy()


def quat_xyzw_to_matrix(q):
    x, y, z, w = np.asarray(q, dtype=float)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not 0.8 < n < 1.2:
        raise ValueError("invalid controller orientation")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat_xyzw(matrix):
    # MuJoCo's quaternion format is wxyz; ROS uses xyzw.
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(matrix, dtype=float).reshape(9))
    return q[[1, 2, 3, 0]]


class QuestInput(Node):
    def __init__(self, sides=("left", "right")):
        super().__init__("quest_aloha_sim")
        self.lock = threading.Lock()
        self.samples = {
            side: {"pose": None, "pose_time": 0.0, "grip": 0.0, "grip_time": 0.0, "trigger": 0.0}
            for side in ("left", "right")
        }
        self.quest_subscriptions = []
        self.pose_pubs = {}
        self.target_pubs = {}
        self.error_pubs = {}
        self.engaged_pubs = {}
        self.gripper_pubs = {}
        for side in sides:
            self.quest_subscriptions.append(
                self.create_subscription(
                    PoseStamped, f"/quest/{side}/pose", lambda m, s=side: self.on_pose(s, m), 10
                )
            )
            for control in ("grip", "trigger"):
                self.quest_subscriptions.append(
                    self.create_subscription(
                        Float32,
                        f"/quest/{side}/{control}",
                        lambda m, s=side, c=control: self.on_analog(s, c, m),
                        10,
                    )
                )
            self.pose_pubs[side] = self.create_publisher(
                PoseStamped, f"/aloha_sim/{side}/ee_pose", 10
            )
            self.target_pubs[side] = self.create_publisher(
                PoseStamped, f"/aloha_sim/{side}/ee_target", 10
            )
            self.error_pubs[side] = self.create_publisher(
                Float32, f"/aloha_sim/{side}/position_error", 10
            )
            self.engaged_pubs[side] = self.create_publisher(
                Bool, f"/aloha_sim/{side}/engaged", 10
            )
            self.gripper_pubs[side] = self.create_publisher(
                Float32, f"/aloha_sim/{side}/gripper_position", 10
            )

    def on_pose(self, side, msg):
        if msg.header.frame_id != "quest_world_openxr":
            return
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation = np.array(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation)):
            return
        try:
            rotation = quat_xyzw_to_matrix(orientation)
        except ValueError:
            return
        with self.lock:
            self.samples[side]["pose"] = (position, rotation)
            self.samples[side]["pose_time"] = time.monotonic()

    def on_analog(self, side, control, msg):
        if not math.isfinite(msg.data):
            return
        with self.lock:
            self.samples[side][control] = float(np.clip(msg.data, 0.0, 1.0))
            if control == "grip":
                self.samples[side]["grip_time"] = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {
                side: {
                    **sample,
                    "pose": None
                    if sample["pose"] is None
                    else (sample["pose"][0].copy(), sample["pose"][1].copy()),
                }
                for side, sample in self.samples.items()
            }

    def publish_sim_state(self, model, data, arms):
        stamp = self.get_clock().now().to_msg()
        for side, arm in arms.items():
            site = data.site(f"{side}/gripper")
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = FRAME_ID
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = site.xpos
            (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ) = matrix_to_quat_xyzw(site.xmat)
            self.pose_pubs[side].publish(msg)
            target = PoseStamped()
            target.header.stamp = stamp
            target.header.frame_id = FRAME_ID
            target.pose.position.x, target.pose.position.y, target.pose.position.z = arm.target_position
            (
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ) = matrix_to_quat_xyzw(arm.target_rotation)
            self.target_pubs[side].publish(target)
            self.error_pubs[side].publish(
                Float32(data=float(np.linalg.norm(site.xpos - arm.target_position)))
            )
            self.engaged_pubs[side].publish(Bool(data=arm.engaged))
            self.gripper_pubs[side].publish(Float32(data=float(data.ctrl[arm.gripper_actuator])))


class SimArm:
    def __init__(self, model, data, side, single_arm=False):
        self.side = side
        self.single_arm = single_arm
        self.site_id = model.site(f"{side}/gripper").id
        self.joint_ids = [model.joint(f"{side}/{name}").id for name in ARM_JOINTS]
        self.qpos_ids = np.array([model.jnt_qposadr[j] for j in self.joint_ids], dtype=int)
        self.dof_ids = np.array([model.jnt_dofadr[j] for j in self.joint_ids], dtype=int)
        self.actuator_ids = np.array(
            [model.actuator(f"{side}/{name}").id for name in ARM_JOINTS], dtype=int
        )
        self.gripper_actuator = model.actuator(f"{side}/gripper").id
        self.finger_qpos_ids = np.array(
            [model.jnt_qposadr[model.joint(f"{side}/{finger}_finger").id] for finger in ("left", "right")],
            dtype=int,
        )
        self.joint_min = model.jnt_range[self.joint_ids, 0] + 0.02
        self.joint_max = model.jnt_range[self.joint_ids, 1] - 0.02
        self.home_position = data.site(f"{side}/gripper").xpos.copy()
        self.target_position = self.home_position.copy()
        self.target_rotation = data.site(f"{side}/gripper").xmat.reshape(3, 3).copy()
        self.engaged = False
        self.rebases = 0
        self.controller_origin = None
        self.controller_rotation_origin = None
        self.ee_origin = None
        self.ee_rotation_origin = None
        self.previous_controller_position = None
        self.jacp = np.zeros((3, model.nv))
        self.jacr = np.zeros((3, model.nv))

    def release(self, data):
        if self.engaged:
            site = data.site(f"{self.side}/gripper")
            self.target_position = site.xpos.copy()
            self.target_rotation = site.xmat.reshape(3, 3).copy()
        self.engaged = False
        self.controller_origin = None
        self.previous_controller_position = None

    def update_target(self, sample, data, now):
        pose_fresh = sample["pose"] is not None and now - sample["pose_time"] < TRACKING_TIMEOUT
        grip_fresh = now - sample["grip_time"] < TRACKING_TIMEOUT
        grip_held = grip_fresh and sample["grip"] >= GRIP_THRESHOLD
        if not grip_held:
            self.release(data)
            return
        if not pose_fresh:
            self.release(data)
            return
        position, rotation = sample["pose"]
        if not self.engaged:
            site = data.site(f"{self.side}/gripper")
            self.controller_origin = position.copy()
            self.controller_rotation_origin = rotation.copy()
            self.ee_origin = site.xpos.copy()
            self.ee_rotation_origin = site.xmat.reshape(3, 3).copy()
            self.previous_controller_position = position.copy()
            self.engaged = True
            return
        displacement = position - self.controller_origin
        frame_jump = np.linalg.norm(position - self.previous_controller_position)
        self.previous_controller_position = position.copy()
        if frame_jump > MAX_FRAME_JUMP:
            self.release(data)
            self.rebases += 1
            return
        displacement_norm = np.linalg.norm(displacement)
        if displacement_norm > MAX_REFERENCE_DISPLACEMENT:
            displacement *= MAX_REFERENCE_DISPLACEMENT / displacement_norm
        target = self.ee_origin + POSITION_GAIN * QUEST_TRANSLATION_TO_WORLD @ displacement
        target[2] = max(target[2], 0.12)
        if self.single_arm:
            target[0] = float(np.clip(target[0], -0.45, 0.45))
            target[1] = float(np.clip(target[1], -0.10, 0.65))
        else:
            target[0] = min(target[0], -0.04) if self.side == "left" else max(target[0], 0.04)
            target[1] = float(np.clip(target[1], -0.30, 0.28))
        self.target_position = target
        controller_delta = rotation @ self.controller_rotation_origin.T
        world_delta = QUEST_ROTATION_TO_WORLD @ controller_delta @ QUEST_ROTATION_TO_WORLD.T
        # Spherical interpolation from identity to the controller rotation delta.
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, world_delta.reshape(9))
        if q[0] < 0:
            q = -q
        angle = 2 * math.atan2(np.linalg.norm(q[1:]), q[0])
        angle = min(angle * ROTATION_GAIN, 0.65)
        if angle > 1e-8:
            axis = q[1:] / np.linalg.norm(q[1:])
            c, s = math.cos(angle), math.sin(angle)
            skew = np.array(
                [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
            )
            scaled_delta = np.eye(3) + s * skew + (1 - c) * skew @ skew
            self.target_rotation = scaled_delta @ self.ee_rotation_origin

    def control(self, model, data, trigger):
        site = data.site(f"{self.side}/gripper")
        current_rotation = site.xmat.reshape(3, 3)
        position_error = self.target_position - site.xpos
        rotation_error = 0.5 * sum(
            np.cross(current_rotation[:, i], self.target_rotation[:, i]) for i in range(3)
        )
        error = np.r_[position_error * 2.5, rotation_error * 1.4]
        mujoco.mj_jacSite(model, data, self.jacp, self.jacr, self.site_id)
        jacobian = np.vstack((self.jacp[:, self.dof_ids], self.jacr[:, self.dof_ids]))
        damping = 0.06
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping**2 * np.eye(6), error
        )
        delta = np.clip(delta, -0.012, 0.012)
        desired = np.clip(data.qpos[self.qpos_ids] + delta, self.joint_min, self.joint_max)
        data.ctrl[self.actuator_ids] = desired
        data.qpos[self.qpos_ids] = desired
        low, high = model.actuator_ctrlrange[self.gripper_actuator]
        finger_position = high - np.clip(trigger, 0.0, 1.0) * (high - low)
        data.ctrl[self.gripper_actuator] = finger_position
        data.qpos[self.finger_qpos_ids] = finger_position


def save_snapshot(model, data, path):
    renderer = mujoco.Renderer(model, height=480, width=640)
    renderer.update_scene(data, camera="teleoperator_pov")
    pixels = renderer.render()
    path = Path(path)
    # PPM avoids an image library dependency and is easy to convert/view.
    path.write_bytes(b"P6\n640 480\n255\n" + pixels.tobytes())
    renderer.close()


def run(args):
    if args.calibration:
        load_calibration(args.calibration)
    single_arm = args.scene == "single"
    sides = ("right",) if single_arm else ("left", "right")
    model = mujoco.MjModel.from_xml_path(str(SINGLE_MODEL_PATH if single_arm else MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    arms = {side: SimArm(model, data, side, single_arm=single_arm) for side in sides}
    rclpy.init()
    node = QuestInput(sides)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    viewer = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(model, data)
        if single_arm:
            with viewer.lock():
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = model.camera("teleoperator_pov").id
    node.get_logger().info(
        f"{args.scene} scene ready; active controllers: {sides}. Hold grip to move; index trigger closes gripper."
    )
    start = time.monotonic()
    last_publish = start
    last_log = start
    last_viewer_sync = start
    try:
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            loop_start = time.monotonic()
            if args.seconds and loop_start - start >= args.seconds:
                break
            samples = node.snapshot()
            for side, arm in arms.items():
                sample = samples[side]
                arm.update_target(sample, data, loop_start)
                trigger = sample["trigger"] if loop_start - sample["pose_time"] < TRACKING_TIMEOUT else 0.0
                arm.control(model, data, trigger)
            # Kinematic stepping keeps the teleop mapping stable while we verify IK.
            # Contact forces and actuator tracking will be tested in a later stage.
            data.qvel[:] = 0
            data.time += 0.01
            mujoco.mj_forward(model, data)
            if viewer is not None and loop_start - last_viewer_sync >= 1 / 30:
                viewer.sync()
                last_viewer_sync = loop_start
            if loop_start - last_publish >= 0.05:
                node.publish_sim_state(model, data, arms)
                last_publish = loop_start
            if loop_start - last_log >= 5.0:
                for side, arm in arms.items():
                    node.get_logger().info(
                        f"{side}={'engaged' if arm.engaged else 'held'} "
                        f"ee={np.round(data.site(f'{side}/gripper').xpos, 3)} rebases={arm.rebases}"
                    )
                last_log = loop_start
            remaining = 0.01 - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if args.snapshot:
            save_snapshot(model, data, args.snapshot)
        if viewer is not None:
            viewer.close()
        executor.shutdown()
        spinner.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("--calibration", type=Path, help="apply a saved axis calibration JSON")
    argument_parser.add_argument("--scene", choices=("single", "dual"), default="single",
                                 help="single forward-facing ViperX (default), or original dual ALOHA")
    argument_parser.add_argument("--headless", action="store_true", help="run without the MuJoCo window")
    argument_parser.add_argument("--seconds", type=float, default=0, help="0 runs until the window closes")
    argument_parser.add_argument("--snapshot", help="save a PPM image of the final simulated pose")
    run(argument_parser.parse_args())
