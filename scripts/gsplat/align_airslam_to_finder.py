#!/usr/bin/env python3
"""Align AirSLAM's world frame to finder's world frame, transform map points.

Inputs:
  airslam_traj: TUM file of AirSLAM keyframe poses (world = AirSLAM's own init)
  airslam_ply:  .ply of AirSLAM map points (same world)
  finder_traj:  TUM file of finder poses (world = LIO's start)

Output:
  <out>/map_points_in_finder.ply — AirSLAM points after applying T_finder_airslam
  <out>/alignment.json — the 4×4 transform + residual APE

Method: for each AirSLAM KF timestamp, find nearest finder pose (tolerance 0.05s),
then run Umeyama SE3 (rigid, scale=1) on the paired points.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def load_tum(path):
    data = np.loadtxt(path, comments="#", dtype=float)
    t = data[:, 0]
    xyz = data[:, 1:4]
    quat_xyzw = data[:, 4:8]
    return t, xyz, quat_xyzw


def umeyama(src, tgt, allow_scale=False):
    """Find (s, R, t) s.t. s * R @ src.T + t ≈ tgt.T. With allow_scale=False, s=1."""
    mu_s = src.mean(0)
    mu_t = tgt.mean(0)
    src_c = src - mu_s
    tgt_c = tgt - mu_t
    H = src_c.T @ tgt_c
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = (Vt.T @ D @ U.T)
    if allow_scale:
        var_s = (src_c ** 2).sum() / len(src_c)
        s = (np.diag(np.eye(3) @ D) * np.linalg.svd(H, compute_uv=False)).sum() / (len(src_c) * var_s)
    else:
        s = 1.0
    t = mu_t - s * R @ mu_s
    return s, R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--airslam_traj", required=True)
    ap.add_argument("--airslam_ply", required=True)
    ap.add_argument("--finder_traj", required=True)
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--t_tol", type=float, default=0.05)
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    t_a, p_a, _ = load_tum(args.airslam_traj)
    t_f, p_f, _ = load_tum(args.finder_traj)
    print(f"[load] airslam KFs={len(t_a)}  finder poses={len(t_f)}")

    # Pair by nearest timestamp
    idx_f = np.searchsorted(t_f, t_a)
    idx_f = np.clip(idx_f, 1, len(t_f) - 1)
    prev_close = np.abs(t_f[idx_f - 1] - t_a) < np.abs(t_f[idx_f] - t_a)
    idx_f[prev_close] -= 1
    dt = np.abs(t_f[idx_f] - t_a)
    keep = dt < args.t_tol
    src = p_a[keep]
    tgt = p_f[idx_f[keep]]
    print(f"[pair] {len(src)}/{len(t_a)} KF paired within {args.t_tol}s")

    # Try SE3 first
    _, R_se3, t_se3 = umeyama(src, tgt, allow_scale=False)
    err_se3 = np.linalg.norm((R_se3 @ src.T).T + t_se3 - tgt, axis=1)
    # Try Sim3
    s_sim, R_sim, t_sim = umeyama(src, tgt, allow_scale=True)
    err_sim = np.linalg.norm((s_sim * R_sim @ src.T).T + t_sim - tgt, axis=1)
    print(f"[align] SE3 rmse={np.sqrt((err_se3 ** 2).mean()):.3f}m  "
          f"Sim3 rmse={np.sqrt((err_sim ** 2).mean()):.3f}m scale={s_sim:.4f}")
    # Pick better
    if err_sim.mean() < err_se3.mean() * 0.9:
        print("[align] using Sim3 (meaningful scale)")
        R, t, s = R_sim, t_sim, s_sim
        err = err_sim
    else:
        print("[align] using SE3 (scale ~= 1)")
        R, t, s = R_se3, t_se3, 1.0
        err = err_se3
    print(f"[align] residual APE mean={err.mean():.3f}m  max={err.max():.3f}m  rmse={np.sqrt((err ** 2).mean()):.3f}m")

    T = np.eye(4); T[:3, :3] = s * R; T[:3, 3] = t

    # Transform the map .ply
    ply_lines = open(args.airslam_ply).readlines()
    header_end = next(i for i, line in enumerate(ply_lines) if line.strip() == "end_header") + 1
    header = ply_lines[:header_end]
    rows = ply_lines[header_end:]
    N = len(rows)
    xyz = np.zeros((N, 3)); rgb = np.zeros((N, 3), dtype=np.uint8)
    for i, line in enumerate(rows):
        parts = line.split()
        xyz[i] = [float(parts[0]), float(parts[1]), float(parts[2])]
        if len(parts) >= 6:
            rgb[i] = [int(parts[3]), int(parts[4]), int(parts[5])]
    xyz_t = (s * R @ xyz.T).T + t
    print(f"[points] transformed {N} map points")

    # Filter to trajectory bbox + padding (avoid stray far points)
    pad_xy, pad_z = 10.0, 6.0
    bbox_lo = tgt.min(0) - np.array([pad_xy, pad_xy, pad_z])
    bbox_hi = tgt.max(0) + np.array([pad_xy, pad_xy, pad_z])
    mask = np.all((xyz_t >= bbox_lo) & (xyz_t <= bbox_hi), axis=1)
    n_in, n_out = int(mask.sum()), int((~mask).sum())
    print(f"[filter] kept {n_in} inliers, dropped {n_out} outliers "
          f"(finder traj bbox±{pad_xy}/{pad_z}m)")
    xyz_t = xyz_t[mask]
    rgb = rgb[mask]
    N = len(xyz_t)

    out_ply = out / "map_points_in_finder.ply"
    with open(out_ply, "w") as f:
        # Rewrite header with new vertex count
        for line in header:
            if line.startswith("element vertex"):
                f.write(f"element vertex {N}\n")
            else:
                f.write(line)
        for i in range(N):
            f.write(f"{xyz_t[i, 0]:.6f} {xyz_t[i, 1]:.6f} {xyz_t[i, 2]:.6f} "
                    f"{rgb[i, 0]} {rgb[i, 1]} {rgb[i, 2]}\n")

    meta = {
        "T_finder_airslam_row_major": T.flatten().tolist(),
        "residual_mean_m": float(err.mean()),
        "residual_rmse_m": float(np.sqrt((err ** 2).mean())),
        "residual_max_m": float(err.max()),
        "paired_count": int(keep.sum()),
        "total_airslam_kf": len(t_a),
        "bbox_airslam": [xyz.min(0).tolist(), xyz.max(0).tolist()],
        "bbox_finder_frame": [xyz_t.min(0).tolist(), xyz_t.max(0).tolist()],
    }
    (out / "alignment.json").write_text(json.dumps(meta, indent=2))
    print(f"[out] {out_ply}")
    print(f"[out] {out / 'alignment.json'}")
    print(f"bbox in finder frame: {meta['bbox_finder_frame']}")


if __name__ == "__main__":
    main()
