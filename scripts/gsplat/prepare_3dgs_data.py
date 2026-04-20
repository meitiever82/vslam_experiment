#!/usr/bin/env python3
"""
Prepare gsplat training data from GeoScan B1 bag + finder_localization.txt.

For each of N sub-sampled finder poses, find the nearest cam0 image in the bag
(by timestamp), undistort fisheye → pinhole, and write:

    <out>/
        images/NNNNNN.jpg        # undistorted pinhole
        poses.json               # per-frame cam2world (4x4) + timestamp
        intrinsics.json          # pinhole K + (W,H)

Later the trainer picks everything up from this folder.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader


def load_tum(path):
    arr = np.loadtxt(path, comments="#")
    return arr[:, 0], arr[:, 1:4], arr[:, 4:8]  # t, xyz, quat(xyzw)


def quat_xyzw_to_R(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def image_msg_to_bgr(msg):
    H, W = msg.height, msg.width
    enc = msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "rgb8":
        return cv2.cvtColor(buf.reshape(H, W, 3), cv2.COLOR_RGB2BGR)
    if enc == "bgr8":
        return buf.reshape(H, W, 3).copy()
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(H, W), cv2.COLOR_GRAY2BGR)
    raise RuntimeError(f"unsupported encoding {enc}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--calib", required=True, help="Kalibr camchain YAML")
    ap.add_argument("--poses", required=True, help="TUM trajectory (finder)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--topic", default="/left_camera/image")
    ap.add_argument("--num_frames", type=int, default=300, help="sub-sampled view count")
    ap.add_argument("--balance", type=float, default=0.0)
    ap.add_argument("--jpeg_quality", type=int, default=92)
    args = ap.parse_args()

    out = Path(args.out).expanduser().resolve()
    img_dir = out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    with open(args.calib) as f:
        calib = yaml.safe_load(f)
    cam0 = calib["cam0"]
    assert cam0["distortion_model"] == "equidistant"
    fx, fy, cx, cy = cam0["intrinsics"]
    D = np.array(cam0["distortion_coeffs"]).reshape(4, 1)
    W, H = cam0["resolution"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, (W, H), np.eye(3), balance=args.balance
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K_new, (W, H), cv2.CV_16SC2
    )
    print(f"[calib] pinhole K (after undistort, balance={args.balance}):\n{K_new}")

    # finder_localization.txt is the IMU (body) pose in world. To get cam0 pose:
    #   T_world_cam0 = T_world_imu @ T_imu_cam0
    #   T_imu_cam0 = inverse(T_cam_imu)  [Kalibr stores T_cam_imu]
    T_cam0_imu = np.array(cam0["T_cam_imu"], dtype=np.float64)
    T_imu_cam0 = np.linalg.inv(T_cam0_imu)
    print(f"[calib] T_imu_cam0 (applied to finder poses):\n{T_imu_cam0}")

    # --- Subsample finder poses ---
    t_arr, xyz_arr, q_arr = load_tum(args.poses)
    N = len(t_arr)
    print(f"[poses] finder: {N} poses  t=[{t_arr[0]:.3f}, {t_arr[-1]:.3f}]")
    idx = np.linspace(0, N - 1, args.num_frames).astype(int)
    sampled_t = t_arr[idx]
    sampled_xyz = xyz_arr[idx]
    sampled_q = q_arr[idx]
    print(f"[poses] sub-sampled to {len(idx)} poses")

    # --- Match bag images to sampled pose timestamps ---
    # Pre-scan bag for all msg timestamps (1 pass), then binary search per pose.
    bag_ts_list = []
    bag_msg_list = []  # (rawdata, conn.msgtype) — we'll deserialize on-demand in pass 2
    with AnyReader([Path(args.bag).expanduser().resolve()]) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            raise RuntimeError(f"topic {args.topic} not in bag")
        # Pass 1: collect timestamps only
        print(f"[bag] scanning timestamps...")
        for conn, bag_ns, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            bag_ts_list.append(ts)
        bag_ts = np.array(bag_ts_list)
        print(f"[bag] {len(bag_ts)} images  t=[{bag_ts[0]:.3f}, {bag_ts[-1]:.3f}]")

        # For each sampled pose, find nearest bag image
        # (linear pass with pointer — both arrays monotonic)
        sorted_j = np.searchsorted(bag_ts, sampled_t)
        nearest = np.clip(sorted_j, 1, len(bag_ts) - 1)
        left_better = np.abs(bag_ts[nearest - 1] - sampled_t) < np.abs(
            bag_ts[nearest] - sampled_t
        )
        nearest = np.where(left_better, nearest - 1, nearest)
        unique_nearest = sorted(set(int(x) for x in nearest))
        if len(unique_nearest) < len(nearest):
            print(
                f"[match] WARN: {len(nearest) - len(unique_nearest)} poses collapsed to same frame"
            )

        # Pass 2: extract only the matched frames
        print(f"[bag] re-scanning to extract {len(unique_nearest)} frames...")
        match_idx_to_kept = {j: k for k, j in enumerate(unique_nearest)}
        kept_images = [None] * len(unique_nearest)
        raw_i = 0
        for conn, bag_ns, rawdata in reader.messages(connections=conns):
            if raw_i in match_idx_to_kept:
                msg = reader.deserialize(rawdata, conn.msgtype)
                bgr = image_msg_to_bgr(msg)
                undist = cv2.remap(
                    bgr, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT
                )
                kept_images[match_idx_to_kept[raw_i]] = undist
            raw_i += 1

    # --- Write images + poses ---
    # Re-map poses to kept frames (drop collapsed ones, keep their first occurrence)
    frame_entries = []
    kept_set = set(unique_nearest)
    seen_kept = set()
    for pose_i, nj in enumerate(nearest):
        nj = int(nj)
        if nj in seen_kept:
            continue
        seen_kept.add(nj)
        kept_slot = unique_nearest.index(nj)
        img = kept_images[kept_slot]
        fname = f"{kept_slot:06d}.jpg"
        cv2.imwrite(
            str(img_dir / fname), img, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
        )
        R = quat_xyzw_to_R(sampled_q[pose_i])
        t = sampled_xyz[pose_i]
        T_world_imu = np.eye(4)
        T_world_imu[:3, :3] = R
        T_world_imu[:3, 3] = t
        # finder pose is IMU in world → convert to camera in world
        T = T_world_imu @ T_imu_cam0
        frame_entries.append(
            {
                "file": f"images/{fname}",
                "timestamp": float(sampled_t[pose_i]),
                "T_cam2world": T.tolist(),
            }
        )
    print(f"[out] wrote {len(frame_entries)} (image, pose) pairs")

    (out / "poses.json").write_text(json.dumps({"frames": frame_entries}, indent=2))
    (out / "intrinsics.json").write_text(
        json.dumps(
            {"K": K_new.tolist(), "W": int(W), "H": int(H), "model": "pinhole"},
            indent=2,
        )
    )
    # Also dump trajectory bbox + center for gaussian init
    pts = np.array([e["T_cam2world"] for e in frame_entries])[:, :3, 3]
    bbox_min = pts.min(0).tolist()
    bbox_max = pts.max(0).tolist()
    center = pts.mean(0).tolist()
    (out / "scene_stats.json").write_text(
        json.dumps(
            {
                "num_frames": len(frame_entries),
                "traj_bbox_min": bbox_min,
                "traj_bbox_max": bbox_max,
                "traj_center": center,
                "traj_extent": float(np.linalg.norm(np.array(bbox_max) - np.array(bbox_min))),
            },
            indent=2,
        )
    )
    print(f"[out] traj extent {np.linalg.norm(np.array(bbox_max) - np.array(bbox_min)):.2f} m")
    print(f"[out] {out}")


if __name__ == "__main__":
    main()
