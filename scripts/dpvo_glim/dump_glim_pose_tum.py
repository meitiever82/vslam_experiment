#!/usr/bin/env python3
"""Subscribe to GLIM's optimized pose topic and append to a TUM-format file.

Usage:
    python3 dump_glim_pose_tum.py \\
        --topic /glim_ros/pose_corrected \\
        --out /tmp/glim_with_dpvo.tum
"""
import argparse
import signal
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TumDumper(Node):
    def __init__(self, topic, out_path):
        super().__init__('glim_tum_dumper')
        self.f = open(out_path, 'w')
        self.f.write('# TUM format: timestamp tx ty tz qx qy qz qw (GLIM pose)\n')
        self.count = 0
        self.sub = self.create_subscription(PoseStamped, topic, self.cb, 100)
        self.get_logger().info(f'subscribed to {topic} → {out_path}')

    def cb(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        o = msg.pose.orientation
        self.f.write(
            f'{ts:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
            f'{o.x:.6f} {o.y:.6f} {o.z:.6f} {o.w:.6f}\n'
        )
        self.f.flush()
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'dumped {self.count} poses')

    def close(self):
        self.f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/glim_ros/pose_corrected')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rclpy.init()
    node = TumDumper(args.topic, args.out)

    def shutdown(sig, frame):
        node.get_logger().info(f'shutting down, total poses: {node.count}')
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        rclpy.spin(node)
    finally:
        node.close()


if __name__ == '__main__':
    main()
