#!/usr/bin/env python3
"""
3DGS trainer with gsplat DefaultStrategy (densify + prune).

Much closer to paper quality than the minimal trainer — adds dynamic split/clone
of high-gradient gaussians and pruning of transparent/oversized ones. L1 + SSIM
(manual, no torchmetrics dependency) loss.

Input folder layout same as prepare_3dgs_data.py output.
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
from torch import nn

from gsplat import rasterization
from gsplat.strategy import DefaultStrategy


def load_data(data_dir):
    data_dir = Path(data_dir)
    with open(data_dir / "poses.json") as f:
        poses = json.load(f)["frames"]
    with open(data_dir / "intrinsics.json") as f:
        K_json = json.load(f)
    with open(data_dir / "scene_stats.json") as f:
        stats = json.load(f)
    K_full = np.array(K_json["K"], dtype=np.float32)
    W, H = int(K_json["W"]), int(K_json["H"])
    frames = []
    for frm in poses:
        img = cv2.imread(str(data_dir / frm["file"]))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        T = np.array(frm["T_cam2world"], dtype=np.float32)
        frames.append({"img": img, "T_c2w": T, "ts": frm["timestamp"]})
    print(f"[data] {len(frames)} frames, full res {W}x{H}")
    return frames, K_full, W, H, stats


def _load_sfm_ply(path):
    """Read xyz + rgb from a 3DGS-style ascii .ply (columns x y z r g b)."""
    xyz, rgb = [], []
    with open(path) as f:
        header = True
        for line in f:
            if header:
                if line.strip() == "end_header":
                    header = False
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            xyz.append([float(parts[0]), float(parts[1]), float(parts[2])])
            if len(parts) >= 6:
                rgb.append([int(parts[3]), int(parts[4]), int(parts[5])])
            else:
                rgb.append([128, 128, 128])
    return np.array(xyz, dtype=np.float32), np.array(rgb, dtype=np.float32) / 255.0


def _knn_mean_dist(xyz, k=3):
    """Per-point mean distance to k nearest neighbors (paper's 3DGS scale init)."""
    from scipy.spatial import cKDTree
    tree = cKDTree(xyz)
    d, _ = tree.query(xyz, k=k + 1)   # +1 because first hit is self
    return d[:, 1:].mean(axis=1)


def init_splats(num_points, bbox_min, bbox_max, device, mean_color, pad_xy, pad_z,
                sfm_ply_path=None):
    bbox_min = np.array(bbox_min, dtype=np.float32)
    bbox_max = np.array(bbox_max, dtype=np.float32)
    lo = bbox_min - np.array([pad_xy, pad_xy, pad_z], dtype=np.float32)
    hi = bbox_max + np.array([pad_xy, pad_xy, pad_z], dtype=np.float32)
    print(
        f"[init] bbox x=[{lo[0]:.1f},{hi[0]:.1f}] "
        f"y=[{lo[1]:.1f},{hi[1]:.1f}] z=[{lo[2]:.1f},{hi[2]:.1f}]"
    )

    if sfm_ply_path is not None:
        xyz, rgb = _load_sfm_ply(sfm_ply_path)
        print(f"[sfm_init] loaded {len(xyz)} points from {sfm_ply_path}")
        # Per-point scale = log(mean kNN distance * 0.5) — paper heuristic
        knn = _knn_mean_dist(xyz, k=3).astype(np.float32)
        knn = np.clip(knn * 0.5, 0.02, 2.0)       # sensible floor/ceiling in meters
        means = torch.from_numpy(xyz).to(device)
        scales_np = np.stack([np.log(knn)] * 3, axis=-1)
        scales = torch.from_numpy(scales_np).to(device)
        colors_rgb = torch.from_numpy(rgb).to(device).clamp(1e-3, 1 - 1e-3)
        # If the source ply has all-gray points (e.g. map_to_ply.cpp didn't fill
        # colors), fall back to per-view mean so we don't seed a gray scene.
        if torch.allclose(colors_rgb, colors_rgb[0]):
            print("[sfm_init] all points same color — falling back to mean_color for init")
            colors_rgb = mean_color.to(device).clamp(1e-3, 1 - 1e-3).unsqueeze(0).expand_as(colors_rgb)
        colors = torch.logit(colors_rgb).contiguous().clone()
        num_points = len(xyz)
    else:
        means = torch.rand(num_points, 3, device=device) * (
            torch.tensor(hi - lo, device=device, dtype=torch.float32)
        ) + torch.tensor(lo, device=device, dtype=torch.float32)
        # log-scale initialized so actual scale = 0.1 m (single-pixel-ish at 10m)
        scales = torch.full((num_points, 3), math.log(0.1), device=device)
        dc = torch.logit(mean_color.to(device).clamp(1e-3, 1 - 1e-3))
        colors = dc.unsqueeze(0).expand(num_points, 3).contiguous().clone()

    quats = torch.zeros(num_points, 4, device=device)
    quats[:, 0] = 1.0
    opacities = torch.full(
        (num_points,), torch.logit(torch.tensor(0.1)).item(), device=device
    )
    # SH DC convention (3DGS paper) — we parameterize as logit colors and apply
    # sigmoid at render time; save_ply() maps to f_dc at write time.

    splats = nn.ParameterDict(
        {
            "means": nn.Parameter(means.requires_grad_(True)),
            "scales": nn.Parameter(scales.requires_grad_(True)),
            "quats": nn.Parameter(quats.requires_grad_(True)),
            "opacities": nn.Parameter(opacities.requires_grad_(True)),
            "sh0": nn.Parameter(colors.requires_grad_(True)),  # SH DC term (3-ch)
        }
    )
    return splats


def build_optimizers(splats, lrs):
    return {
        name: torch.optim.Adam([p], lr=lrs[name], eps=1e-15)
        for name, p in splats.items()
    }


_SSIM_KERNEL_CACHE = {}


def _gaussian_kernel(window_size: int, sigma: float, device, dtype):
    key = (window_size, sigma, device, dtype)
    k = _SSIM_KERNEL_CACHE.get(key)
    if k is None:
        ax = torch.arange(window_size, device=device, dtype=dtype) - (window_size - 1) / 2
        g = torch.exp(-(ax ** 2) / (2 * sigma ** 2))
        g = g / g.sum()
        k2d = g.unsqueeze(1) * g.unsqueeze(0)
        k = k2d.expand(3, 1, window_size, window_size).contiguous()
        _SSIM_KERNEL_CACHE[key] = k
    return k


def ssim(img1, img2, window_size: int = 11, sigma: float = 1.5):
    """Windowed SSIM on RGB. img: (H, W, 3) in [0,1]. Returns mean SSIM in [0,1]."""
    x = img1.permute(2, 0, 1).unsqueeze(0)  # (1,3,H,W)
    y = img2.permute(2, 0, 1).unsqueeze(0)
    k = _gaussian_kernel(window_size, sigma, x.device, x.dtype)
    pad = window_size // 2
    mu_x = torch.nn.functional.conv2d(x, k, padding=pad, groups=3)
    mu_y = torch.nn.functional.conv2d(y, k, padding=pad, groups=3)
    mu_x2, mu_y2, mu_xy = mu_x * mu_x, mu_y * mu_y, mu_x * mu_y
    sigma_x2 = torch.nn.functional.conv2d(x * x, k, padding=pad, groups=3) - mu_x2
    sigma_y2 = torch.nn.functional.conv2d(y * y, k, padding=pad, groups=3) - mu_y2
    sigma_xy = torch.nn.functional.conv2d(x * y, k, padding=pad, groups=3) - mu_xy
    c1, c2 = 0.01 ** 2, 0.03 ** 2
    ssim_map = ((2 * mu_xy + c1) * (2 * sigma_xy + c2)) / (
        (mu_x2 + mu_y2 + c1) * (sigma_x2 + sigma_y2 + c2)
    )
    return ssim_map.mean()


def render_one(splats, viewmat, K, W, H, scale=1):
    Wr, Hr = W // scale, H // scale
    Kr = K.clone()
    Kr[0, 0] /= scale
    Kr[1, 1] /= scale
    Kr[0, 2] /= scale
    Kr[1, 2] /= scale
    means = splats["means"]
    scales_t = torch.exp(splats["scales"])
    quats = splats["quats"] / splats["quats"].norm(dim=-1, keepdim=True)
    opacs = torch.sigmoid(splats["opacities"])
    colors = torch.sigmoid(splats["sh0"])
    renders, alphas, info = rasterization(
        means,
        quats,
        scales_t,
        opacs,
        colors,
        viewmat[None],
        Kr[None],
        Wr,
        Hr,
        packed=False,
    )
    return renders[0], alphas[0], info, Kr, Wr, Hr


def save_ply(path, splats):
    m = splats["means"].detach().cpu().numpy()
    s = splats["scales"].detach().cpu().numpy()  # log scale
    q = splats["quats"].detach().cpu().numpy()
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    op = splats["opacities"].detach().cpu().numpy()  # logit
    sh = splats["sh0"].detach().cpu().numpy()  # logit color
    N = m.shape[0]
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {N}\n")
        for nm in ("x", "y", "z", "nx", "ny", "nz",
                   "f_dc_0", "f_dc_1", "f_dc_2", "opacity",
                   "scale_0", "scale_1", "scale_2",
                   "rot_0", "rot_1", "rot_2", "rot_3"):
            f.write(f"property float {nm}\n")
        f.write("end_header\n")
        # Map our sigmoid-based color to 3DGS SH-DC convention:
        #   color_out = 0.5 + SH(0) * f_dc  with SH(0) = 0.28209479
        # Inverting from sigmoid: c = sigmoid(sh). f_dc = (c - 0.5) / SH(0)
        c = 1.0 / (1.0 + np.exp(-sh))
        f_dc = (c - 0.5) / 0.28209479177387814
        for i in range(N):
            line = [
                m[i, 0], m[i, 1], m[i, 2],
                0.0, 0.0, 0.0,
                f_dc[i, 0], f_dc[i, 1], f_dc[i, 2],
                op[i],
                s[i, 0], s[i, 1], s[i, 2],
                q[i, 0], q[i, 1], q[i, 2], q[i, 3],
            ]
            f.write(" ".join(f"{v:.6f}" for v in line) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--iters", type=int, default=15000)
    ap.add_argument("--num_points", type=int, default=100_000)
    ap.add_argument("--render_scale", type=int, default=2)
    ap.add_argument("--val_every", type=int, default=2000)
    ap.add_argument("--pad_xy", type=float, default=8.0)
    ap.add_argument("--pad_z", type=float, default=6.0)
    ap.add_argument("--ssim_lambda", type=float, default=0.2,
                    help="loss = (1-λ)·L1 + λ·(1-SSIM); 3DGS paper uses 0.2")
    ap.add_argument("--sfm_init", type=str, default=None,
                    help="path to .ply whose xyz (+optional rgb) are used as initial gaussians")
    args = ap.parse_args()

    torch.manual_seed(42)
    random.seed(42)
    np.random.seed(42)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    out = Path(args.out).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    log_f = open(out / "train_log.txt", "w")

    frames, K_full, W, H, stats = load_data(args.data)
    scale = args.render_scale
    Wr, Hr = W // scale, H // scale
    print(f"[render] {Wr}x{Hr}")
    gt_gpu = []
    for frm in frames:
        img_rs = cv2.resize(frm["img"], (Wr, Hr), interpolation=cv2.INTER_AREA)
        gt_gpu.append(torch.from_numpy(img_rs).to(device))
    viewmats = [
        torch.linalg.inv(torch.from_numpy(frm["T_c2w"])).to(device) for frm in frames
    ]
    K = torch.from_numpy(K_full).to(device)

    mean_color = torch.stack([g.mean(dim=(0, 1)) for g in gt_gpu]).mean(0).cpu()
    print(f"[init] mean image color: {mean_color.tolist()}")
    splats = init_splats(
        args.num_points, stats["traj_bbox_min"], stats["traj_bbox_max"], device,
        mean_color, args.pad_xy, args.pad_z,
        sfm_ply_path=args.sfm_init,
    )
    print(f"[init] N_initial = {splats['means'].shape[0]}")

    # Per-param lrs — 3DGS paper defaults (roughly)
    scene_scale = float(stats["traj_extent"]) / 4.0
    lrs = {
        "means": 1.6e-4 * scene_scale,
        "scales": 5e-3,
        "quats": 1e-3,
        "opacities": 5e-2,
        "sh0": 2.5e-3,
    }
    optimizers = build_optimizers(splats, lrs)

    strategy = DefaultStrategy(
        prune_opa=0.005,
        grow_grad2d=0.0002,
        refine_start_iter=500,
        refine_stop_iter=min(args.iters - 500, 15000),
        refine_every=100,
        reset_every=3000,
        verbose=False,
    )
    strategy.check_sanity(splats, optimizers)
    state = strategy.initialize_state(scene_scale=scene_scale)

    print(f"[train] {args.iters} iters, {args.num_points} init gaussians, {len(frames)} views, scene_scale={scene_scale:.2f}")
    t0 = time.time()
    ema_loss = None
    for it in range(args.iters):
        idx = random.randint(0, len(frames) - 1)
        gt = gt_gpu[idx]
        vm = viewmats[idx]

        render_rgb, _, info, _, _, _ = render_one(splats, vm, K, W, H, scale=scale)
        l1 = torch.nn.functional.l1_loss(render_rgb, gt)
        dssim = 1.0 - ssim(render_rgb, gt)
        loss = (1.0 - args.ssim_lambda) * l1 + args.ssim_lambda * dssim

        strategy.step_pre_backward(
            params=splats, optimizers=optimizers, state=state, step=it, info=info
        )
        for opt in optimizers.values():
            opt.zero_grad()
        loss.backward()
        strategy.step_post_backward(
            params=splats, optimizers=optimizers, state=state, step=it, info=info
        )
        for opt in optimizers.values():
            opt.step()

        loss_v = loss.item()
        l1_v = l1.item()
        ema_loss = loss_v if ema_loss is None else 0.98 * ema_loss + 0.02 * loss_v
        if (it + 1) % 200 == 0:
            with torch.no_grad():
                mse = torch.nn.functional.mse_loss(render_rgb, gt).item()
            psnr = -10 * math.log10(mse + 1e-12)
            N = splats["means"].shape[0]
            elapsed = time.time() - t0
            line = (f"iter={it + 1:6d}/{args.iters} loss={loss_v:.5f} "
                    f"l1={l1_v:.5f} ema={ema_loss:.5f} psnr={psnr:.2f} "
                    f"N={N} {elapsed:.0f}s")
            print(line)
            log_f.write(line + "\n")
            log_f.flush()

        if (it + 1) % args.val_every == 0 or it == args.iters - 1:
            with torch.no_grad():
                val_idx = len(frames) // 2
                r, _, _, _, _, _ = render_one(splats, viewmats[val_idx], K, W, H, scale=scale)
                im = (r.clamp(0, 1).detach().cpu().numpy() * 255).astype(np.uint8)
                cv2.imwrite(str(out / f"render_val_{it + 1:06d}.jpg"),
                            cv2.cvtColor(im, cv2.COLOR_RGB2BGR))

    save_ply(out / "point_cloud.ply", splats)
    peak_mem = torch.cuda.max_memory_allocated() / 1024**3 if device == "cuda" else 0
    N_final = splats["means"].shape[0]

    # Final eval pass: mean PSNR/SSIM over all training views
    # (no held-out split — matches benchmark convention in 06-benchmarks.md)
    psnrs, ssims = [], []
    with torch.no_grad():
        for idx in range(len(frames)):
            r, _, _, _, _, _ = render_one(splats, viewmats[idx], K, W, H, scale=scale)
            r = r.clamp(0, 1)
            gt = gt_gpu[idx]
            mse = torch.nn.functional.mse_loss(r, gt).item()
            psnrs.append(-10 * math.log10(mse + 1e-12))
            ssims.append(ssim(r, gt).item())
    mean_psnr = float(np.mean(psnrs))
    mean_ssim = float(np.mean(ssims))
    elapsed = time.time() - t0
    print(f"[done] {elapsed:.0f}s, peak VRAM={peak_mem:.2f} GB, final N={N_final}")
    print(f"[eval] mean PSNR={mean_psnr:.2f} dB, mean SSIM={mean_ssim:.4f} over {len(frames)} views")
    print(f"[out] {out / 'point_cloud.ply'}")
    log_f.write(f"[done] {elapsed:.0f}s peak_vram={peak_mem:.2f}GB N={N_final} "
                f"psnr={mean_psnr:.2f} ssim={mean_ssim:.4f}\n")
    log_f.close()


if __name__ == "__main__":
    main()
