"""ROS 2 node that wraps DROID-W (DROID-SLAM in the Wild).

Workflow
--------
1. Subscribe to a monocular image topic (sensor_msgs/Image, optionally
   CompressedImage). Every accepted frame is pushed into a FrameBuffer with
   its ROS timestamp converted to seconds.
2. On a std_srvs/Trigger ``~/start`` service call (or after auto_start_after_n
   frames have arrived), the buffer is frozen, wrapped in a BufferedRGB
   dataset, and SLAM is launched in a background Python thread.
3. While SLAM is running, a wall timer reads keyframe poses from the
   shared-memory ``slam.video`` and publishes
       - the latest keyframe pose as geometry_msgs/PoseStamped
       - the latest keyframe pose as nav_msgs/Odometry
       - the running keyframe trajectory as nav_msgs/Path
       - a TF transform from ``world_frame_id`` to ``camera_frame_id``.
4. Once slam.run() returns, the timer keeps publishing the final trajectory.
   On node shutdown, the latest trajectory is also dumped as a TUM-format
   text file (``output_traj_path``) for offline use.

Coordinate convention
---------------------
``DepthVideo.poses`` stores world-to-camera SE3 elements as
``[tx, ty, tz, qx, qy, qz, qw]`` (lietorch convention). Before publishing we
invert each element to get camera-in-world, which is what TF / nav_msgs/Path
/ nav_msgs/Odometry expect.
"""

from __future__ import annotations

import os
import sys
import threading
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from droid_w_ros2.frame_buffer import FrameBuffer

PoseTuple = Tuple[float, float, float, float, float, float, float]  # tx,ty,tz,qx,qy,qz,qw


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_inverse(qx: float, qy: float, qz: float, qw: float):
    return -qx, -qy, -qz, qw  # unit quaternion inverse = conjugate


def _quat_rotate(q, v):
    """Rotate 3-vector v by quaternion q = (x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def _invert_w2c(pose7) -> PoseTuple:
    """Invert (tx, ty, tz, qx, qy, qz, qw) world-to-camera into camera-in-world."""
    tx, ty, tz, qx, qy, qz, qw = (float(x) for x in pose7)
    iqx, iqy, iqz, iqw = _quat_inverse(qx, qy, qz, qw)
    rtx, rty, rtz = _quat_rotate((iqx, iqy, iqz, iqw), (-tx, -ty, -tz))
    return rtx, rty, rtz, iqx, iqy, iqz, iqw


class DroidWNode(Node):
    def __init__(self):
        super().__init__('droid_w_node')

        # ---- parameters --------------------------------------------------
        self.declare_parameter('droid_root', '')
        self.declare_parameter('droid_config', '')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('compressed', False)
        self.declare_parameter('frame_skip', 1)            # use every Nth frame
        self.declare_parameter('max_frames', 0)            # 0 = unbounded
        self.declare_parameter('auto_start_after_n', 0)    # 0 = manual via service
        self.declare_parameter('world_frame_id', 'droid_world')
        self.declare_parameter('camera_frame_id', 'droid_camera')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('output_traj_path', '')     # '' = skip TUM dump
        # Override cfg['data']['output'] / cfg['scene']. Both default to ''
        # (= keep what the YAML provides). If output_dir is empty AND the YAML
        # holds a relative path, the SLAM output will land under the cwd of
        # whoever launched ros2, which is rarely what you want — set this.
        self.declare_parameter('output_dir', '')
        self.declare_parameter('scene', 'ros_session')

        droid_root = self.get_parameter('droid_root').get_parameter_value().string_value
        self.droid_config_path = self.get_parameter('droid_config').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.compressed = self.get_parameter('compressed').get_parameter_value().bool_value
        self.frame_skip = max(1, int(self.get_parameter('frame_skip').get_parameter_value().integer_value))
        self.max_frames = int(self.get_parameter('max_frames').get_parameter_value().integer_value)
        self.auto_start_after_n = int(self.get_parameter('auto_start_after_n').get_parameter_value().integer_value)
        self.world_frame_id = self.get_parameter('world_frame_id').get_parameter_value().string_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        publish_rate = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        self.output_traj_path = self.get_parameter('output_traj_path').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.scene = self.get_parameter('scene').get_parameter_value().string_value

        if not droid_root or not os.path.isdir(droid_root):
            raise RuntimeError(
                f"Parameter 'droid_root' must point to the DROID-W repo root; got: {droid_root!r}")
        if not self.droid_config_path or not os.path.isfile(self.droid_config_path):
            raise RuntimeError(
                f"Parameter 'droid_config' must point to a DROID-W YAML config; got: {self.droid_config_path!r}")

        # Make `from src...` imports work for the SLAM modules.
        if droid_root not in sys.path:
            sys.path.insert(0, droid_root)

        # ---- runtime state -----------------------------------------------
        self.bridge = CvBridge()
        self.buffer = FrameBuffer(max_frames=self.max_frames)
        self._frame_counter = 0
        self._slam = None
        self._slam_thread: Optional[threading.Thread] = None
        self._slam_done = threading.Event()
        # Sticky flag: once SLAM has been launched (or attempted) we never
        # try again, even if the launch failed. Prevents auto_start_after_n
        # from re-firing on every subsequent frame after a setup error.
        self._slam_started = False
        # Cache image_timestamps from the BufferedRGB stream so the publish
        # tick can map kf frame index → wall-clock time.
        self._stream_timestamps: Optional[np.ndarray] = None
        # latest snapshot of (timestamp_sec, camera-in-world pose tuple)
        self._latest_traj: List[Tuple[float, PoseTuple]] = []
        self._latest_lock = threading.Lock()

        # ---- I/O ---------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        if self.compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, image_topic, self._on_compressed_image, sensor_qos)
        else:
            self.image_sub = self.create_subscription(
                Image, image_topic, self._on_image, sensor_qos)

        self.path_pub = self.create_publisher(Path, '~/kf_path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '~/kf_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '~/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_srv = self.create_service(Trigger, '~/start', self._on_start)
        self.stop_srv = self.create_service(Trigger, '~/stop', self._on_stop)

        period = 1.0 / max(0.1, publish_rate)
        self.publish_timer = self.create_timer(period, self._on_publish_tick)

        self.get_logger().info(
            f"droid_w_node ready. topic={image_topic} compressed={self.compressed} "
            f"frame_skip={self.frame_skip} max_frames={self.max_frames} "
            f"auto_start_after_n={self.auto_start_after_n}")

    # ---- subscription callbacks ------------------------------------------
    def _on_image(self, msg: Image) -> None:
        if self._slam_started:
            return  # SLAM already launched on a frozen buffer
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failure: {e}")
            return
        self._handle_frame(_stamp_to_sec(msg.header.stamp), cv_img)

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        if self._slam_started:
            return
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failure (compressed): {e}")
            return
        self._handle_frame(_stamp_to_sec(msg.header.stamp), cv_img)

    def _handle_frame(self, ts: float, bgr: np.ndarray) -> None:
        self._frame_counter += 1
        if (self._frame_counter % self.frame_skip) != 0:
            return
        if not self.buffer.push(ts, bgr):
            return  # buffer frozen → ignore
        if (self.auto_start_after_n > 0
                and len(self.buffer) >= self.auto_start_after_n
                and not self._slam_started):
            self.get_logger().info(
                f"auto-start triggered: collected {len(self.buffer)} frames")
            self._launch_slam()

    # ---- services --------------------------------------------------------
    def _on_start(self, request, response):
        if self._slam_started:
            response.success = False
            response.message = 'SLAM already running or already attempted'
            return response
        n = len(self.buffer)
        if n == 0:
            response.success = False
            response.message = 'No frames buffered yet'
            return response
        self._launch_slam()
        response.success = True
        response.message = f'launched SLAM on {n} frames'
        return response

    def _on_stop(self, request, response):
        # Best-effort: we cannot kill the spawned tracker process from here,
        # but we can stop accepting new frames.
        self.buffer.freeze()
        response.success = True
        response.message = f'buffer frozen at {len(self.buffer)} frames'
        return response

    # ---- SLAM launch -----------------------------------------------------
    def _launch_slam(self) -> None:
        # Mark started up-front so a launch failure doesn't make us retry on
        # every subsequent frame.
        self._slam_started = True

        frames = self.buffer.freeze()
        if not frames:
            self.get_logger().error("buffer empty, cannot launch SLAM")
            return

        # Imported here so missing torch / CUDA only fail when SLAM is
        # actually requested (lets us at least bring the node up).
        from src import config as droid_config_module
        from src.slam import SLAM
        from src.utils.common import setup_seed
        from src.utils.datasets import BufferedRGB

        cfg = droid_config_module.load_config(self.droid_config_path)
        cfg['dataset'] = 'ros'
        cfg['scene'] = self.scene
        if self.output_dir:
            cfg.setdefault('data', {})['output'] = self.output_dir
        # Disable GUI / rerun viewer in the ROS path; the node publishes its
        # own visualisations.
        cfg['gui'] = False
        cfg['droidvis'] = False
        # Match run.py: in fast_mode the final refine iterations are forced
        # down. Use direct assignment, NOT setdefault — the YAML already has
        # a (much larger) value so setdefault would be a no-op.
        if cfg.get('fast_mode', False):
            cfg.setdefault('mapping', {})['final_refine_iters'] = 3000

        setup_seed(cfg.get('setup_seed', 43))

        try:
            stream = BufferedRGB(cfg, frames)
        except Exception as e:
            self.get_logger().error(f"failed to build BufferedRGB: {e}")
            return

        # Cache the wall-clock timestamps so the publish tick can map
        # video.timestamp[idx] (= source frame index) → real seconds.
        self._stream_timestamps = np.asarray(stream.image_timestamps, dtype=np.float64)

        self.get_logger().info(
            f"constructing SLAM with {len(frames)} frames, "
            f"scene={cfg['scene']}, output={cfg['data']['output']}")
        try:
            self._slam = SLAM(cfg, stream)
        except Exception as e:
            self.get_logger().error(f"failed to construct SLAM: {e}")
            return
        self._slam_done.clear()
        self._slam_thread = threading.Thread(
            target=self._slam_run, name='droid_w_slam', daemon=True)
        self._slam_thread.start()

    def _slam_run(self) -> None:
        try:
            self._slam.run()
        except Exception as e:
            self.get_logger().error(f"slam.run() raised: {e}")
        finally:
            self._slam_done.set()
            self.get_logger().info("slam.run() finished")

    # ---- pose publishing -------------------------------------------------
    def _snapshot_keyframes(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self._slam is None:
            return None
        video = getattr(self._slam, 'video', None)
        if video is None:
            return None
        try:
            n = int(video.counter.value)
        except Exception:
            return None
        if n <= 0:
            return None
        try:
            poses_w2c = video.poses[:n].detach().clone().cpu().numpy()
            timestamps = video.timestamp[:n].detach().clone().cpu().numpy()
        except Exception as e:
            self.get_logger().warn(f"failed to snapshot poses: {e}")
            return None
        return poses_w2c, timestamps

    def _on_publish_tick(self) -> None:
        snap = self._snapshot_keyframes()
        if snap is None:
            return
        poses_w2c, kf_frame_idx = snap
        n = len(poses_w2c)
        # Map keyframe → wall-clock seconds. video.timestamp[k] is the SOURCE
        # frame index in the BufferedRGB stream (see eval_traj.py:27 which
        # casts to int and uses it as an index). We look up the cached
        # image_timestamps to recover the original ROS time.
        stream_ts = self._stream_timestamps

        traj: List[Tuple[float, PoseTuple]] = []
        path_msg = Path()
        now_msg = self.get_clock().now().to_msg()
        path_msg.header.stamp = now_msg
        path_msg.header.frame_id = self.world_frame_id

        latest_pose: Optional[PoseStamped] = None
        for i in range(n):
            tx, ty, tz, qx, qy, qz, qw = _invert_w2c(poses_w2c[i])
            ps = PoseStamped()
            ps.header.frame_id = self.world_frame_id
            idx = int(kf_frame_idx[i])
            if stream_ts is not None and 0 <= idx < len(stream_ts):
                sec = float(stream_ts[idx])
            else:
                sec = float(idx)  # fallback: at least monotonically increasing
            ps.header.stamp.sec = int(sec)
            ps.header.stamp.nanosec = int((sec - int(sec)) * 1e9)
            ps.pose.position.x = tx
            ps.pose.position.y = ty
            ps.pose.position.z = tz
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_msg.poses.append(ps)
            latest_pose = ps
            traj.append((sec, (tx, ty, tz, qx, qy, qz, qw)))

        with self._latest_lock:
            self._latest_traj = traj

        self.path_pub.publish(path_msg)
        if latest_pose is not None:
            self.pose_pub.publish(latest_pose)

            odom = Odometry()
            odom.header.stamp = now_msg
            odom.header.frame_id = self.world_frame_id
            odom.child_frame_id = self.camera_frame_id
            odom.pose.pose = latest_pose.pose
            self.odom_pub.publish(odom)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_msg
            tf_msg.header.frame_id = self.world_frame_id
            tf_msg.child_frame_id = self.camera_frame_id
            tf_msg.transform.translation.x = latest_pose.pose.position.x
            tf_msg.transform.translation.y = latest_pose.pose.position.y
            tf_msg.transform.translation.z = latest_pose.pose.position.z
            tf_msg.transform.rotation = latest_pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    # ---- shutdown --------------------------------------------------------
    def dump_tum_trajectory(self) -> None:
        if not self.output_traj_path:
            return
        with self._latest_lock:
            traj = list(self._latest_traj)
        if not traj:
            self.get_logger().info("no trajectory to dump")
            return
        out_dir = os.path.dirname(self.output_traj_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.output_traj_path, 'w') as f:
            for ts, (tx, ty, tz, qx, qy, qz, qw) in traj:
                f.write(
                    f"{ts:.6f} {tx:.6f} {ty:.6f} {tz:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
        self.get_logger().info(
            f"wrote {len(traj)} keyframe poses to {self.output_traj_path}")


def main(args=None):
    rclpy.init(args=args)
    node = DroidWNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.dump_tum_trajectory()
        except Exception as e:  # never let dump break shutdown
            node.get_logger().error(f"trajectory dump failed: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
