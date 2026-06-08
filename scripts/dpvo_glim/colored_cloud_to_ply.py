#!/usr/bin/env python3
"""Accumulate /colored_cloud PointCloud2 frames into a single PLY file.

Subscribes to /colored_cloud, appends all points to a growing file.  Stops
on Ctrl-C and writes final PLY with accumulated points.
"""
import argparse
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class Accumulator(Node):
    def __init__(self, topic, out_path, stride_pts):
        super().__init__("colored_cloud_accumulator")
        self.out_path = out_path
        self.stride_pts = stride_pts
        self.buf_xyz = []  # list of Nx3
        self.buf_rgb = []  # list of Nx3 uint8
        self.sub = self.create_subscription(
            PointCloud2, topic,
            self._cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10),
        )
        self.get_logger().info(f"subscribed to {topic} → {out_path}")
        self._n_msgs = 0

    def _cb(self, msg):
        gen = pc2.read_points(
            msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        rows = list(gen)
        if not rows:
            return
        pts = np.array([[r[0], r[1], r[2], r[3]] for r in rows], dtype=np.float32)
        if self.stride_pts > 1:
            pts = pts[::self.stride_pts]
        xyz = pts[:, :3]
        # Unpack RGB float32 → uint32 → bytes
        rgb_float = pts[:, 3].astype(np.float32)
        rgb_uint = rgb_float.view(np.uint32)
        r = ((rgb_uint >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_uint >> 8) & 0xFF).astype(np.uint8)
        b = (rgb_uint & 0xFF).astype(np.uint8)
        rgb = np.stack([r, g, b], axis=1)
        self.buf_xyz.append(xyz)
        self.buf_rgb.append(rgb)
        self._n_msgs += 1
        total_pts = sum(x.shape[0] for x in self.buf_xyz)
        if self._n_msgs % 20 == 0:
            self.get_logger().info(
                f"accumulated {self._n_msgs} msgs, {total_pts} points")
        # Auto-flush every 50 msgs so we always have a recent PLY on disk
        if self._n_msgs % 50 == 0:
            self.save()

    def save(self):
        if not self.buf_xyz:
            self.get_logger().warn("no data to save")
            return
        xyz = np.concatenate(self.buf_xyz, axis=0)
        rgb = np.concatenate(self.buf_rgb, axis=0)
        self.get_logger().info(
            f"writing {xyz.shape[0]} points to {self.out_path}...")
        with open(self.out_path, "wb") as f:
            header = (
                f"ply\nformat binary_little_endian 1.0\n"
                f"element vertex {xyz.shape[0]}\n"
                f"property float x\nproperty float y\nproperty float z\n"
                f"property uchar red\nproperty uchar green\nproperty uchar blue\n"
                f"end_header\n"
            )
            f.write(header.encode())
            dtype = np.dtype([
                ("x", "f4"), ("y", "f4"), ("z", "f4"),
                ("r", "u1"), ("g", "u1"), ("b", "u1"),
            ])
            rec = np.empty(xyz.shape[0], dtype=dtype)
            rec["x"] = xyz[:, 0]
            rec["y"] = xyz[:, 1]
            rec["z"] = xyz[:, 2]
            rec["r"] = rgb[:, 0]
            rec["g"] = rgb[:, 1]
            rec["b"] = rgb[:, 2]
            f.write(rec.tobytes())
        self.get_logger().info("saved.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/colored_cloud")
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=1,
                    help="subsample every Nth point (reduce size)")
    args = ap.parse_args()

    rclpy.init()
    node = Accumulator(args.topic, args.out, args.stride)

    def shutdown(sig, frame):
        node.save()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        rclpy.spin(node)
    finally:
        node.save()


if __name__ == "__main__":
    main()
