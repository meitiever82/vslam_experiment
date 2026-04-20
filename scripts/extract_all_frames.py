#!/usr/bin/env python3
"""
Dump every cam0/cam1 frame from a GeoScan ROS2 bag to disk as JPEG/PNG.

Output layout:
    <out>/
        cam0/<bag_ts_ns>.jpg    (raw fisheye, no undistort)
        cam1/<bag_ts_ns>.jpg
        manifest.csv            (topic, index, header_ts_s, bag_ts_ns, path, W, H)
"""

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader


def msg_to_bgr(msg):
    H, W = msg.height, msg.width
    enc = msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "rgb8":
        return cv2.cvtColor(buf.reshape(H, W, 3), cv2.COLOR_RGB2BGR)
    if enc == "bgr8":
        return buf.reshape(H, W, 3).copy()
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(H, W), cv2.COLOR_GRAY2BGR)
    raise RuntimeError(f"unsupported {enc}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument(
        "--topics",
        nargs="+",
        default=["/left_camera/image", "/right_camera/image"],
    )
    ap.add_argument("--ext", choices=["jpg", "png"], default="jpg")
    ap.add_argument("--jpeg_quality", type=int, default=92)
    args = ap.parse_args()

    out = Path(args.out).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    topic_to_folder = {}
    for t in args.topics:
        name = t.strip("/").split("/")[0]  # "/left_camera/image" -> "left_camera"
        fn = name.replace("_camera", "")  # "left"
        d = out / f"cam_{fn}"
        d.mkdir(exist_ok=True)
        topic_to_folder[t] = d

    manifest_path = out / "manifest.csv"
    mf = open(manifest_path, "w", newline="")
    writer = csv.writer(mf)
    writer.writerow(["topic", "index", "header_ts_s", "bag_ts_ns", "path", "W", "H"])

    per_topic_count = {t: 0 for t in args.topics}
    total = 0
    with AnyReader([Path(args.bag).expanduser().resolve()]) as reader:
        conns = [c for c in reader.connections if c.topic in args.topics]
        if not conns:
            raise RuntimeError(f"none of {args.topics} in bag")
        total_msgs = sum(c.msgcount for c in conns)
        print(f"[bag] {total_msgs} image msgs across {len(conns)} topic(s)")
        for conn, bag_ns, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            bgr = msg_to_bgr(msg)
            ts_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            folder = topic_to_folder[conn.topic]
            fname = f"{bag_ns}.{args.ext}"
            path = folder / fname
            if args.ext == "jpg":
                cv2.imwrite(str(path), bgr, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
            else:
                cv2.imwrite(str(path), bgr)
            writer.writerow([conn.topic, per_topic_count[conn.topic], f"{ts_s:.9f}",
                             bag_ns, str(path.relative_to(out)), msg.width, msg.height])
            per_topic_count[conn.topic] += 1
            total += 1
            if total % 500 == 0:
                print(f"  {total}/{total_msgs} written")
    mf.close()

    print("\n=== summary ===")
    for t, n in per_topic_count.items():
        print(f"  {t}: {n} frames → {topic_to_folder[t]}")
    print(f"  manifest: {manifest_path}")


if __name__ == "__main__":
    main()
