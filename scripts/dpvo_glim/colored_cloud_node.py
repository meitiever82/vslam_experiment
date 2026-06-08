#!/usr/bin/env python3
"""Realtime colored point cloud from GLIM LiDAR + GeoScan cam0.

Subscribes to:
    /glim_ros/aligned_points_corrected (sensor_msgs/PointCloud2, map frame)
    /glim_ros/pose_corrected           (geometry_msgs/PoseStamped, map frame, IMU=base pose)
    /left_camera/image                 (sensor_msgs/Image, cam0 raw fisheye)

Publishes:
    /colored_cloud                     (sensor_msgs/PointCloud2 with XYZ + RGB)

Logic per incoming points msg:
    1. find image + pose with nearest header stamp (small ring buffer)
    2. compute T_map_cam = T_map_imu @ T_imu_cam (T_imu_cam from config)
    3. for each world point p_w:
         p_cam = inv(T_map_cam) @ p_w
         if p_cam.z <= 0: drop (behind camera)
         (u, v) = fisheye_project(p_cam, K_orig, D_equidistant)
         if out of image: drop
         rgb = image[v, u]
    4. publish colored cloud
"""
import argparse
import json
import struct
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2 as pc2


def se3_from_tumvec(vec):
    tx, ty, tz, qx, qy, qz, qw = vec
    # quaternion → rotation matrix (x,y,z,w order)
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    R = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
        [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
        [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def pose_msg_to_matrix(msg):
    p = msg.pose.position
    o = msg.pose.orientation
    return se3_from_tumvec([p.x, p.y, p.z, o.x, o.y, o.z, o.w])


def load_camera_from_config(config_dir):
    with open(Path(config_dir) / "config_sensors.json") as f:
        # json with C-style comments — strip them
        import re
        raw = f.read()
        raw = re.sub(r"//[^\n]*", "", raw)
        raw = re.sub(r"/\*.*?\*/", "", raw, flags=re.DOTALL)
        cfg = json.loads(raw)
    s = cfg["sensors"]
    T_lidar_imu = se3_from_tumvec(s["T_lidar_imu"])
    T_lidar_cam = se3_from_tumvec(s["T_lidar_camera"])
    T_imu_cam = np.linalg.inv(T_lidar_imu) @ T_lidar_cam
    K = np.array([
        [s["intrinsics"][0], 0.0, s["intrinsics"][2]],
        [0.0, s["intrinsics"][1], s["intrinsics"][3]],
        [0.0, 0.0, 1.0],
    ])
    D = np.array(s["distortion_coeffs"], dtype=np.float64).reshape(-1)
    assert s["distortion_model"] == "equidistant"
    W, H = int(s["image_size"][0]), int(s["image_size"][1])
    return T_imu_cam, K, D, W, H


def sensor_qos(depth=50):
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class ColoredCloudNode(Node):
    def __init__(self, config_dir, points_topic, pose_topic, image_topic,
                 out_topic, max_time_diff=0.2, max_points_per_msg=80000):
        super().__init__("colored_cloud_node")

        self.T_imu_cam, self.K, self.D, self.W, self.H = load_camera_from_config(config_dir)
        self.T_cam_imu = np.linalg.inv(self.T_imu_cam)
        self.max_time_diff = max_time_diff
        self.max_points = max_points_per_msg

        self.get_logger().info(
            f"K={self.K.tolist()}, D={self.D.tolist()}, size={self.W}x{self.H}")
        self.get_logger().info(f"T_imu_cam translation={self.T_imu_cam[:3, 3]}")

        self.image_buf = deque(maxlen=8)  # (stamp, bgr)
        self.pose_buf = deque(maxlen=200)  # (stamp, T_map_imu)

        self.sub_img = self.create_subscription(
            Image, image_topic, self._cb_image, sensor_qos())
        self.sub_pose = self.create_subscription(
            PoseStamped, pose_topic, self._cb_pose, sensor_qos())
        self.sub_pts = self.create_subscription(
            PointCloud2, points_topic, self._cb_points, sensor_qos())
        self.pub_colored = self.create_publisher(
            PointCloud2, out_topic, sensor_qos(depth=5))
        self.get_logger().info(
            f"subscribing: img={image_topic} pose={pose_topic} pts={points_topic}")
        self.get_logger().info(f"publishing: {out_topic}")
        self._n_processed = 0

    @staticmethod
    def _stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _cb_image(self, msg):
        H, W = msg.height, msg.width
        enc = msg.encoding
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ("rgb8", "bgr8"):
            img = buf.reshape(H, W, 3)
            if enc == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif enc == "mono8":
            img = cv2.cvtColor(buf.reshape(H, W), cv2.COLOR_GRAY2BGR)
        else:
            return
        self.image_buf.append((self._stamp_to_sec(msg.header.stamp), img))

    def _cb_pose(self, msg):
        self.pose_buf.append(
            (self._stamp_to_sec(msg.header.stamp), pose_msg_to_matrix(msg)))

    def _find_nearest(self, buf, stamp):
        if not buf:
            return None
        best = min(buf, key=lambda x: abs(x[0] - stamp))
        if abs(best[0] - stamp) > self.max_time_diff:
            return None
        return best[1]

    def _cb_points(self, msg):
        stamp = self._stamp_to_sec(msg.header.stamp)
        img_pair = self._find_nearest(self.image_buf, stamp)
        pose_pair = self._find_nearest(self.pose_buf, stamp)
        if img_pair is None or pose_pair is None:
            return  # no fresh image/pose to color with

        image = img_pair
        T_map_imu = pose_pair
        T_cam_map = self.T_cam_imu @ np.linalg.inv(T_map_imu)

        # Extract xyz from incoming cloud (drop intensity/other fields)
        pts_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        try:
            pts = np.array([[p[0], p[1], p[2]] for p in pts_gen], dtype=np.float32)
        except Exception:
            pts = np.asarray(list(pts_gen), dtype=np.float32)
            if pts.size and pts.ndim == 1:
                pts = pts.view(np.float32).reshape(-1, 3)
        if pts.shape[0] == 0:
            return
        if pts.shape[0] > self.max_points:
            idx = np.random.choice(pts.shape[0], self.max_points, replace=False)
            pts = pts[idx]

        # Transform to camera frame
        ones = np.ones((pts.shape[0], 1), dtype=np.float32)
        pts_h = np.concatenate([pts, ones], axis=1)  # Nx4
        pts_cam = (T_cam_map @ pts_h.T).T[:, :3]

        # Keep points in front of camera
        in_front = pts_cam[:, 2] > 0.1
        pts_cam = pts_cam[in_front]
        pts_world_kept = pts[in_front]
        if pts_cam.shape[0] == 0:
            return

        # Fisheye project: cv2.fisheye.projectPoints expects Nx1x3 input
        pts_in = pts_cam.reshape(-1, 1, 3).astype(np.float64)
        rvec = np.zeros((3, 1))
        tvec = np.zeros((3, 1))
        uv, _ = cv2.fisheye.projectPoints(pts_in, rvec, tvec, self.K, self.D)
        uv = uv.reshape(-1, 2)

        # Filter by image bounds
        u = uv[:, 0]
        v = uv[:, 1]
        in_img = (u >= 0) & (u < self.W - 1) & (v >= 0) & (v < self.H - 1)
        if not in_img.any():
            return
        u_i = u[in_img].astype(np.int32)
        v_i = v[in_img].astype(np.int32)
        pts_out = pts_world_kept[in_img]
        bgr = image[v_i, u_i]  # Nx3 uint8 BGR

        # Build PointCloud2 with XYZ + RGB packed
        r = bgr[:, 2].astype(np.uint32)
        g = bgr[:, 1].astype(np.uint32)
        b = bgr[:, 0].astype(np.uint32)
        rgb_int = (r << 16) | (g << 8) | b
        rgb_float = rgb_int.view(np.float32)

        # Field layout: x, y, z (float32), rgb (float32, packed uint32)
        cloud_data = np.zeros(pts_out.shape[0],
                              dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"),
                                     ("rgb", "f4")])
        cloud_data["x"] = pts_out[:, 0].astype(np.float32)
        cloud_data["y"] = pts_out[:, 1].astype(np.float32)
        cloud_data["z"] = pts_out[:, 2].astype(np.float32)
        cloud_data["rgb"] = rgb_float

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        out_msg = PointCloud2()
        out_msg.header = msg.header  # inherit stamp + frame_id (= map)
        out_msg.height = 1
        out_msg.width = cloud_data.shape[0]
        out_msg.fields = fields
        out_msg.is_bigendian = False
        out_msg.point_step = 16
        out_msg.row_step = 16 * cloud_data.shape[0]
        out_msg.data = cloud_data.tobytes()
        out_msg.is_dense = True
        self.pub_colored.publish(out_msg)

        self._n_processed += 1
        if self._n_processed % 20 == 0:
            self.get_logger().info(
                f"colored cloud #{self._n_processed} ({cloud_data.shape[0]} pts)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config_dir", default=(
        "/home/steve/casbot_ws/src/finder_lidar_mapping/glim/config/geoscan_b1"))
    ap.add_argument("--points_topic", default="/glim_ros/aligned_points_corrected")
    ap.add_argument("--pose_topic", default="/glim_ros/pose_corrected")
    ap.add_argument("--image_topic", default="/left_camera/image")
    ap.add_argument("--out_topic", default="/colored_cloud")
    ap.add_argument("--max_time_diff", type=float, default=0.2)
    args = ap.parse_args()

    rclpy.init()
    node = ColoredCloudNode(
        args.config_dir, args.points_topic, args.pose_topic,
        args.image_topic, args.out_topic,
        max_time_diff=args.max_time_diff,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
