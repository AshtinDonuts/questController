"""Measure a labeled Quest controller translation without moving the simulator."""

import argparse
from datetime import datetime
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class Recorder(Node):
    def __init__(self, side):
        super().__init__("quest_motion_recorder")
        self.samples = []
        self.subscription = self.create_subscription(
            PoseStamped, f"/quest/{side}/pose", self.record, 10
        )

    def record(self, msg):
        if msg.header.frame_id != "quest_world_openxr":
            return
        p = msg.pose.position
        xyz = np.array([p.x, p.y, p.z], dtype=float)
        if np.all(np.isfinite(xyz)):
            self.samples.append((time.monotonic(), xyz))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--motion", default="up", help="Physical movement label for the report")
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args.side)
    try:
        print("Keep the controller STILL for 2 seconds; recording starts now.", flush=True)
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        print(f"Move the {args.side} controller {args.motion.upper()} about 10 cm now. You have 10 seconds; holding is helpful but not required.", flush=True)
        while time.monotonic() - start < 12.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        before = np.array([p for t, p in node.samples if start + 0.5 <= t < start + 1.8])
        after = np.array([p for t, p in node.samples if start + 10.5 <= t < start + 11.8])
        if len(before) < 5 or len(after) < 5:
            raise RuntimeError(f"Insufficient Quest frames: before={len(before)}, after={len(after)}")
        a = np.median(before, axis=0)
        b = np.median(after, axis=0)
        delta = b - a
        trajectory = np.array([p for _, p in node.samples])
        offsets = trajectory - a
        distances = np.linalg.norm(offsets, axis=1)
        peak_index = int(np.argmax(distances))
        peak = offsets[peak_index]
        span = np.ptp(trajectory, axis=0)
        jump = max((np.linalg.norm(q - p) for (_, p), (_, q) in zip(node.samples, node.samples[1:])), default=0)
        valid = float(distances[peak_index]) >= 0.04 and jump <= 0.12
        report = (
            f"Time: {datetime.now().astimezone().isoformat()}\n"
            f"Side: {args.side}; motion: {args.motion}\n"
            f"Before Quest xyz: {a.round(4)} m\n"
            f"After Quest xyz: {b.round(4)} m\n"
            f"Quest displacement xyz: {delta.round(4)} m\n"
            f"Peak displacement from start xyz: {peak.round(4)} m at {node.samples[peak_index][0]-start:.2f} s\n"
            f"Peak distance from start: {distances[peak_index]:.4f} m\n"
            f"Axis span during recording xyz: {span.round(4)} m\n"
            f"Largest frame jump: {jump:.4f} m\n"
            f"Frames: before={len(before)}, after={len(after)}, total={len(node.samples)}\n"
            f"Result: {'motion captured' if valid else 'inconclusive; repeat with a clear movement'}\n"
        )
        path = Path.home() / "Desktop" / f"quest_motion_{args.side}_{args.motion}.txt"
        trace_path = Path.home() / "Desktop" / f"quest_motion_{args.side}_{args.motion}_trace.csv"
        path.write_text(report, encoding="utf-8")
        trace_path.write_text(
            "seconds,x,y,z\n" + "".join(
                f"{t-start:.4f},{p[0]:.6f},{p[1]:.6f},{p[2]:.6f}\n"
                for t, p in node.samples
            ),
            encoding="utf-8",
        )
        print(report, end="")
        print(f"Saved to: {path} and {trace_path}")
        if jump > 0.12:
            print("WARNING: tracking jumped >0.12 m; repeat after recentering the headset.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
