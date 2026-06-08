#!/usr/bin/env python3
"""Drop-in replacement for `ros2 bag play` that guarantees monotonic delivery.

Bypasses the Humble rosbag2 sqlite3 reader (which has a known multi-threaded
out-of-order delivery bug for GeoScan-style multi-topic high-frequency bags).

Reads the bag with rosbags.AnyReader (single-threaded, deterministic, in
chronological order across all topics), then republishes each message on the
same topic via rclpy.

Supports:
    - /clock publishing (`--clock`)
    - `--rate` speedup/slowdown
    - `--start-offset` (seconds from bag start)
    - `--topics` filter (default: all)

Only republishes known message types.  Extend the MSG_MAP below if you need
more.
"""
import argparse
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rosbags.highlevel import AnyReader

# ROS2 message imports — extend if bag has more types
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, Image, MagneticField
from sensor_msgs.msg import PointField


# Map (rosbag type string) → (ROS2 msg class, serialize fn)
# We serialize the message manually to avoid round-trip conversion loss.


def _imu_to_msg(raw):
    """Copy rosbags Imu fields to rclpy Imu."""
    m = Imu()
    m.header.stamp.sec = int(raw.header.stamp.sec)
    m.header.stamp.nanosec = int(raw.header.stamp.nanosec)
    m.header.frame_id = str(raw.header.frame_id)
    m.angular_velocity.x = float(raw.angular_velocity.x)
    m.angular_velocity.y = float(raw.angular_velocity.y)
    m.angular_velocity.z = float(raw.angular_velocity.z)
    m.linear_acceleration.x = float(raw.linear_acceleration.x)
    m.linear_acceleration.y = float(raw.linear_acceleration.y)
    m.linear_acceleration.z = float(raw.linear_acceleration.z)
    m.orientation.x = float(raw.orientation.x)
    m.orientation.y = float(raw.orientation.y)
    m.orientation.z = float(raw.orientation.z)
    m.orientation.w = float(raw.orientation.w)
    # Covariance arrays — keep zeros if upstream was zero
    m.angular_velocity_covariance = [float(v) for v in raw.angular_velocity_covariance]
    m.linear_acceleration_covariance = [float(v) for v in raw.linear_acceleration_covariance]
    m.orientation_covariance = [float(v) for v in raw.orientation_covariance]
    return m


def _image_to_msg(raw):
    m = Image()
    m.header.stamp.sec = int(raw.header.stamp.sec)
    m.header.stamp.nanosec = int(raw.header.stamp.nanosec)
    m.header.frame_id = str(raw.header.frame_id)
    m.height = int(raw.height)
    m.width = int(raw.width)
    m.encoding = str(raw.encoding)
    m.is_bigendian = int(raw.is_bigendian)
    m.step = int(raw.step)
    m.data = bytes(raw.data)
    return m


def _pc2_to_msg(raw):
    m = PointCloud2()
    m.header.stamp.sec = int(raw.header.stamp.sec)
    m.header.stamp.nanosec = int(raw.header.stamp.nanosec)
    m.header.frame_id = str(raw.header.frame_id)
    m.height = int(raw.height)
    m.width = int(raw.width)
    for f in raw.fields:
        pf = PointField()
        pf.name = str(f.name)
        pf.offset = int(f.offset)
        pf.datatype = int(f.datatype)
        pf.count = int(f.count)
        m.fields.append(pf)
    m.is_bigendian = bool(raw.is_bigendian)
    m.point_step = int(raw.point_step)
    m.row_step = int(raw.row_step)
    m.data = bytes(raw.data)
    m.is_dense = bool(raw.is_dense)
    return m


def _mag_to_msg(raw):
    m = MagneticField()
    m.header.stamp.sec = int(raw.header.stamp.sec)
    m.header.stamp.nanosec = int(raw.header.stamp.nanosec)
    m.header.frame_id = str(raw.header.frame_id)
    m.magnetic_field.x = float(raw.magnetic_field.x)
    m.magnetic_field.y = float(raw.magnetic_field.y)
    m.magnetic_field.z = float(raw.magnetic_field.z)
    m.magnetic_field_covariance = [float(v) for v in raw.magnetic_field_covariance]
    return m


MSG_MAP = {
    "sensor_msgs/msg/Imu": (Imu, _imu_to_msg),
    "sensor_msgs/msg/Image": (Image, _image_to_msg),
    "sensor_msgs/msg/PointCloud2": (PointCloud2, _pc2_to_msg),
    "sensor_msgs/msg/MagneticField": (MagneticField, _mag_to_msg),
}


def sensor_qos(depth=100):
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--start-offset", type=float, default=0.0,
                    help="skip first N seconds")
    ap.add_argument("--clock", action="store_true", help="publish /clock")
    ap.add_argument("--topics", nargs="+", default=None,
                    help="subset of topics to publish (default: all known)")
    args = ap.parse_args()

    rclpy.init()
    node = Node("bag_replay_monotone")

    bag_path = Path(args.bag).expanduser().resolve()
    with AnyReader([bag_path]) as reader:
        # Decide which connections to replay
        conns = reader.connections
        if args.topics is not None:
            conns = [c for c in conns if c.topic in args.topics]
        conns = [c for c in conns if c.msgtype in MSG_MAP]
        if not conns:
            node.get_logger().error("no supported connections selected")
            return

        publishers = {}
        total_by_topic = {}
        for c in conns:
            cls, _ = MSG_MAP[c.msgtype]
            pub = node.create_publisher(cls, c.topic, sensor_qos())
            publishers[c.topic] = (pub, MSG_MAP[c.msgtype][1])
            total_by_topic[c.topic] = c.msgcount
            node.get_logger().info(f"will publish {c.topic} [{c.msgtype}] ({c.msgcount} msgs)")

        clock_pub = None
        if args.clock:
            clock_pub = node.create_publisher(Clock, "/clock", 10)

        # First pass: determine bag start time (header ts of first usable msg)
        start_ts = None
        for conn, bag_ns, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            start_ts = ts
            break
        if start_ts is None:
            node.get_logger().error("bag empty for selected topics")
            return
        node.get_logger().info(f"bag start_ts = {start_ts:.6f}")

        # Streaming sort buffer — rosbags returns messages in BAG_NS order, but
        # different sensors have different capture→publish latency (GeoScan
        # LiDAR has ~99ms jitter, camera ~3ms). To GUARANTEE monotonic header
        # ts to downstream, buffer messages in a min-heap by header ts and
        # only release those older than (max_seen_header_ts - SORT_WINDOW).
        import heapq
        SORT_WINDOW = 0.3  # seconds, safely > 99ms observed jitter
        buf = []  # (header_ts, seq, topic, ros_msg)
        seq = 0  # tiebreaker for heap equal-ts

        play_start = time.time()
        published_by_topic = {t: 0 for t in publishers}
        skipped = 0
        last_published_ts = None
        max_seen_ts = -float("inf")

        def release_one_ready():
            """Pop+publish oldest buffered msg if it's safe (ts < cutoff)."""
            nonlocal last_published_ts
            if not buf:
                return False
            head_ts = buf[0][0]
            if head_ts > max_seen_ts - SORT_WINDOW:
                return False
            _, _, topic, ros_msg = heapq.heappop(buf)
            # Pace by header ts
            target = (head_ts - start_ts - args.start_offset) / args.rate
            elapsed = time.time() - play_start
            if elapsed < target:
                time.sleep(target - elapsed)
            pub, _ = publishers[topic]
            pub.publish(ros_msg)
            if args.clock:
                clk = Clock()
                clk.clock.sec = ros_msg.header.stamp.sec
                clk.clock.nanosec = ros_msg.header.stamp.nanosec
                clock_pub.publish(clk)
            published_by_topic[topic] += 1
            last_published_ts = head_ts
            return True

        for conn, bag_ns, rawdata in reader.messages(connections=conns):
            msg_raw = reader.deserialize(rawdata, conn.msgtype)
            ts = msg_raw.header.stamp.sec + msg_raw.header.stamp.nanosec * 1e-9

            if ts - start_ts < args.start_offset:
                skipped += 1
                continue

            # Convert now so we hold small ROS2 msg in buf, not raw bytes
            _, conv = publishers[conn.topic]
            ros_msg = conv(msg_raw)
            heapq.heappush(buf, (ts, seq, conn.topic, ros_msg))
            seq += 1
            if ts > max_seen_ts:
                max_seen_ts = ts

            # Drain any buffered messages now safe to publish
            while release_one_ready():
                pass

            total = sum(published_by_topic.values())
            if total > 0 and total % 5000 == 0:
                node.get_logger().info(
                    f"published {total} msgs, t={last_published_ts - start_ts - args.start_offset:.1f}s "
                    f"buf={len(buf)}")

        # Drain remaining buffer at end
        while buf:
            _, _, topic, ros_msg = heapq.heappop(buf)
            pub, _ = publishers[topic]
            pub.publish(ros_msg)
            if args.clock:
                clk = Clock()
                clk.clock.sec = ros_msg.header.stamp.sec
                clk.clock.nanosec = ros_msg.header.stamp.nanosec
                clock_pub.publish(clk)
            published_by_topic[topic] += 1

        node.get_logger().info(
            f"done: skipped {skipped}, published " +
            " ".join(f"{t}={n}" for t, n in published_by_topic.items()))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
