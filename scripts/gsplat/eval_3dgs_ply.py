#!/usr/bin/env python3
"""Load a 3DGS .ply and compute mean PSNR/SSIM across all training views.

Used to re-evaluate older runs whose per-iter log PSNR was an L1 approximation,
so we can compare against the real MSE-based PSNR from train_3dgs_densify.py.
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import torch

from gsplat import rasterization

import sys
sys.path.insert(0, str(Path(__file__).parent))
from train_3dgs_densify import load_data, ssim


def load_ply(path, device):
    with open(path, "r") as f:
        header_lines = []
        while True:
            line = f.readline().strip()
            header_lines.append(line)
            if line == "end_header":
                break
        rows = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append([float(v) for v in line.split()])
    arr = np.array(rows, dtype=np.float32)
    # Column layout from save_ply():
    #   0:3 x,y,z   3:6 nx,ny,nz   6:9 f_dc_0..2   9 opacity
    #   10:13 scale_0..2   13:17 rot_0..3
    means = torch.from_numpy(arr[:, 0:3]).to(device)
    f_dc = torch.from_numpy(arr[:, 6:9]).to(device)
    opacities_logit = torch.from_numpy(arr[:, 9]).to(device)
    scales_log = torch.from_numpy(arr[:, 10:13]).to(device)
    quats = torch.from_numpy(arr[:, 13:17]).to(device)
    # Invert save_ply's color mapping: c = sigmoid(sh), f_dc = (c-0.5)/0.28209
    SH_C0 = 0.28209479177387814
    c = (f_dc * SH_C0 + 0.5).clamp(1e-3, 1 - 1e-3)
    # sigmoid input range
    sh0_logit = torch.log(c / (1 - c))
    return {
        "means": means,
        "scales": scales_log,
        "quats": quats,
        "opacities": opacities_logit,
        "sh0": sh0_logit,
    }


def render_one(splats, viewmat, K, W, H, scale):
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
    renders, _, _ = rasterization(
        means, quats, scales_t, opacs, colors,
        viewmat[None], Kr[None], Wr, Hr, packed=False,
    )
    return renders[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--ply", required=True)
    ap.add_argument("--render_scale", type=int, default=2)
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    frames, K_full, W, H, _stats = load_data(args.data)
    Wr, Hr = W // args.render_scale, H // args.render_scale
    gt_gpu = []
    for frm in frames:
        img_rs = cv2.resize(frm["img"], (Wr, Hr), interpolation=cv2.INTER_AREA)
        gt_gpu.append(torch.from_numpy(img_rs).to(device))
    viewmats = [torch.linalg.inv(torch.from_numpy(frm["T_c2w"])).to(device) for frm in frames]
    K = torch.from_numpy(K_full).to(device)

    splats = load_ply(args.ply, device)
    print(f"[ply] N={splats['means'].shape[0]}")

    psnrs, ssims = [], []
    with torch.no_grad():
        for idx in range(len(frames)):
            r = render_one(splats, viewmats[idx], K, W, H, args.render_scale).clamp(0, 1)
            mse = torch.nn.functional.mse_loss(r, gt_gpu[idx]).item()
            psnrs.append(-10 * math.log10(mse + 1e-12))
            ssims.append(ssim(r, gt_gpu[idx]).item())
    print(f"[eval] mean PSNR={np.mean(psnrs):.2f} dB, mean SSIM={np.mean(ssims):.4f} over {len(frames)} views")


if __name__ == "__main__":
    main()
