#!/usr/bin/env python3
"""
Minimal 3DGS trainer for GeoScan 3DGS demo.

Inputs (from prepare_3dgs_data.py output folder):
    images/NNNNNN.jpg
    poses.json               (cam2world, OpenCV)
    intrinsics.json          (pinhole K, WxH)
    scene_stats.json         (bbox + center for gaussian init)

Outputs:
    <out>/
        point_cloud.ply      (final splat, SuperSplat / browser viewer can open)
        train_log.txt        (iter / loss / psnr)
        render_val_*.jpg     (a few validation renders)

Design:
- Random init 100k gaussians inside the trajectory bbox (expanded by 0.3× each side)
- Train loop: random view per iter, MSE + SSIM loss (weighted), Adam
- No densify/prune (gsplat library only; full densify requires examples/trainer.py)
  → trade quality for simplicity. Demo still viewable.
"""

import argparse
import json
import math
import random
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import Tensor

from gsplat import rasterization


def load_data(data_dir):
    data_dir = Path(data_dir)
    with open(data_dir / "poses.json") as f:
        poses = json.load(f)["frames"]
    with open(data_dir / "intrinsics.json") as f:
        K_json = json.load(f)
    with open(data_dir / "scene_stats.json") as f:
        stats = json.load(f)

    K_full = np.array(K_json["K"], dtype=np.float32)
    W_full, H_full = int(K_json["W"]), int(K_json["H"])

    frames = []
    for fi, frm in enumerate(poses):
        img = cv2.imread(str(data_dir / frm["file"]))
        if img is None:
            raise FileNotFoundError(frm["file"])
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        T = np.array(frm["T_cam2world"], dtype=np.float32)
        frames.append({"img": img, "T_c2w": T, "ts": frm["timestamp"], "idx": fi})
    print(f"[data] {len(frames)} frames, full res {W_full}x{H_full}")
    return frames, K_full, W_full, H_full, stats


def init_gaussians(num_points, bbox_min, bbox_max, device, mean_color=None,
                   pad_xy=5.0, pad_z=5.0):
    """Random init inside an expanded bbox around the trajectory.

    Trajectory Z range is often tiny (camera-on-car at fixed height); must pad
    Z strongly to cover ceiling/floor of a parking-garage-like scene.
    """
    bbox_min = np.array(bbox_min, dtype=np.float32)
    bbox_max = np.array(bbox_max, dtype=np.float32)
    lo = bbox_min - np.array([pad_xy, pad_xy, pad_z], dtype=np.float32)
    hi = bbox_max + np.array([pad_xy, pad_xy, pad_z], dtype=np.float32)
    print(f"[init] gaussian bbox x=[{lo[0]:.1f},{hi[0]:.1f}] "
          f"y=[{lo[1]:.1f},{hi[1]:.1f}] z=[{lo[2]:.1f},{hi[2]:.1f}]")

    means = torch.rand(num_points, 3, device=device) * (
        torch.tensor(hi - lo, device=device, dtype=torch.float32)
    ) + torch.tensor(lo, device=device, dtype=torch.float32)

    scales = torch.full((num_points, 3), math.log(0.1), device=device)  # ~0.1 m
    quats = torch.zeros(num_points, 4, device=device)
    quats[:, 0] = 1.0
    opacities = torch.full(
        (num_points,), torch.logit(torch.tensor(0.1)).item(), device=device
    )
    # Init color to scene mean so average render matches ambient brightness
    if mean_color is None:
        mean_color = torch.tensor([0.1, 0.1, 0.1])
    dc_logit = torch.logit(mean_color.clamp(1e-3, 1 - 1e-3))
    rgbs = dc_logit.to(device).unsqueeze(0).expand(num_points, 3).contiguous().clone()
    return means, quats, scales, opacities, rgbs


def to_viewmat(T_c2w):
    """cam2world (OpenCV, +X right +Y down +Z forward) → world2cam."""
    T = torch.from_numpy(T_c2w) if isinstance(T_c2w, np.ndarray) else T_c2w
    return torch.linalg.inv(T)


def render(means, quats, scales, opacities, rgbs, viewmat, K, W, H, scale_factor=1):
    Wr, Hr = W // scale_factor, H // scale_factor
    Kr = K.clone()
    Kr[0, 0] /= scale_factor
    Kr[1, 1] /= scale_factor
    Kr[0, 2] /= scale_factor
    Kr[1, 2] /= scale_factor
    renders, _, _ = rasterization(
        means,
        quats / quats.norm(dim=-1, keepdim=True),
        torch.exp(scales),
        torch.sigmoid(opacities),
        torch.sigmoid(rgbs),
        viewmat[None],
        Kr[None],
        Wr,
        Hr,
        packed=False,
    )
    return renders[0], Kr, Wr, Hr


def save_ply(path, means, quats, scales, opacities, rgbs):
    """Minimal 3DGS PLY (SuperSplat / antimatter15 viewer compatible)."""
    means = means.detach().cpu().numpy()
    quats = (quats / quats.norm(dim=-1, keepdim=True)).detach().cpu().numpy()
    scales = torch.exp(scales).detach().cpu().numpy()
    opacities = torch.sigmoid(opacities).detach().cpu().numpy()
    rgbs = torch.sigmoid(rgbs).detach().cpu().numpy()
    N = means.shape[0]
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {N}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property float f_dc_0\nproperty float f_dc_1\nproperty float f_dc_2\n")
        f.write("property float opacity\n")
        f.write("property float scale_0\nproperty float scale_1\nproperty float scale_2\n")
        f.write("property float rot_0\nproperty float rot_1\nproperty float rot_2\nproperty float rot_3\n")
        f.write("end_header\n")
        # 3DGS convention: f_dc stored as (color - 0.5) / 0.28209... (SH DC term)
        # To keep simple, just write (color*2 - 1) scaled
        dc = (rgbs - 0.5) / 0.28209479177387814
        for i in range(N):
            f.write(
                f"{means[i, 0]} {means[i, 1]} {means[i, 2]} "
                f"{dc[i, 0]} {dc[i, 1]} {dc[i, 2]} "
                f"{np.log(opacities[i] / (1 - opacities[i]) + 1e-9)} "
                f"{np.log(scales[i, 0] + 1e-9)} {np.log(scales[i, 1] + 1e-9)} {np.log(scales[i, 2] + 1e-9)} "
                f"{quats[i, 0]} {quats[i, 1]} {quats[i, 2]} {quats[i, 3]}\n"
            )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--iters", type=int, default=5000)
    ap.add_argument("--num_points", type=int, default=100_000)
    ap.add_argument("--lr_means", type=float, default=1e-4)
    ap.add_argument("--lr_scales", type=float, default=5e-3)
    ap.add_argument("--lr_quats", type=float, default=1e-3)
    ap.add_argument("--lr_opacity", type=float, default=5e-2)
    ap.add_argument("--lr_rgbs", type=float, default=2.5e-2)
    ap.add_argument("--render_scale", type=int, default=2, help="1280/N render size for speed")
    ap.add_argument("--val_every", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--pad_xy", type=float, default=5.0, help="bbox lateral pad [m]")
    ap.add_argument("--pad_z", type=float, default=5.0, help="bbox vertical pad [m] (ceiling/floor)")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    random.seed(args.seed)
    np.random.seed(args.seed)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    out = Path(args.out).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    log_f = open(out / "train_log.txt", "w")

    frames, K_full, W, H, stats = load_data(args.data)

    # Pre-upload full-res GT to GPU, then resize for rendering
    scale = args.render_scale
    Wr, Hr = W // scale, H // scale
    print(f"[render] {Wr}x{Hr} ({W}x{H}/{scale})")
    gt_gpu = []
    for frm in frames:
        img_rs = cv2.resize(frm["img"], (Wr, Hr), interpolation=cv2.INTER_AREA)
        gt_gpu.append(torch.from_numpy(img_rs).to(device))
    viewmats = [to_viewmat(frm["T_c2w"]).to(device) for frm in frames]
    K = torch.from_numpy(K_full).to(device)

    # Compute mean image color for gaussian color init
    mean_color = torch.stack([g.mean(dim=(0, 1)) for g in gt_gpu]).mean(0).cpu()
    print(f"[init] scene mean color: {mean_color.tolist()}")
    means, quats, scales, opacities, rgbs = init_gaussians(
        args.num_points, stats["traj_bbox_min"], stats["traj_bbox_max"], device,
        mean_color=mean_color, pad_xy=args.pad_xy, pad_z=args.pad_z,
    )
    for p in [means, quats, scales, opacities, rgbs]:
        p.requires_grad_(True)

    optim = torch.optim.Adam(
        [
            {"params": [means], "lr": args.lr_means},
            {"params": [scales], "lr": args.lr_scales},
            {"params": [quats], "lr": args.lr_quats},
            {"params": [opacities], "lr": args.lr_opacity},
            {"params": [rgbs], "lr": args.lr_rgbs},
        ]
    )

    print(f"[train] {args.iters} iters, {args.num_points} gaussians, {len(frames)} views")
    t0 = time.time()
    ema_loss = None
    for it in range(args.iters):
        idx = random.randint(0, len(frames) - 1)
        gt = gt_gpu[idx]
        vm = viewmats[idx]

        render_rgb, Kr, _, _ = render(
            means, quats, scales, opacities, rgbs, vm, K, W, H, scale_factor=scale
        )
        loss = torch.nn.functional.mse_loss(render_rgb, gt)
        optim.zero_grad()
        loss.backward()
        optim.step()

        loss_v = loss.item()
        ema_loss = loss_v if ema_loss is None else 0.98 * ema_loss + 0.02 * loss_v
        if (it + 1) % 100 == 0:
            psnr = 10 * math.log10(1.0 / max(ema_loss, 1e-9))
            elapsed = time.time() - t0
            line = f"iter={it + 1:6d}/{args.iters} loss={loss_v:.5f} ema={ema_loss:.5f} psnr={psnr:.2f} {elapsed:.0f}s"
            print(line)
            log_f.write(line + "\n")
            log_f.flush()

        if (it + 1) % args.val_every == 0 or it == args.iters - 1:
            with torch.no_grad():
                val_idx = len(frames) // 2
                r, _, _, _ = render(
                    means, quats, scales, opacities, rgbs,
                    viewmats[val_idx], K, W, H, scale_factor=scale,
                )
                im = (r.clamp(0, 1).detach().cpu().numpy() * 255).astype(np.uint8)
                cv2.imwrite(
                    str(out / f"render_val_{it + 1:06d}.jpg"),
                    cv2.cvtColor(im, cv2.COLOR_RGB2BGR),
                )

    save_ply(out / "point_cloud.ply", means, quats, scales, opacities, rgbs)
    peak_mem = torch.cuda.max_memory_allocated() / 1024**3 if device == "cuda" else 0
    print(f"[done] {time.time() - t0:.0f}s, peak VRAM={peak_mem:.2f} GB")
    print(f"[out] {out / 'point_cloud.ply'}")
    log_f.close()


if __name__ == "__main__":
    main()
