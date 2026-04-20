#!/usr/bin/env python3
"""
Run MapAnything (feedforward metric 3D) on an undistorted GeoScan cam0 image
folder produced by extract_cam0_undist.py.

Inputs (from <input> dir):
    images/NNNNNN.jpg
    timestamps.txt          "idx ts_seconds" per frame

Outputs (to <output> dir):
    trajectory_mapanything.tum     "ts tx ty tz qx qy qz qw" (cam2world, OpenCV)
    scene.glb                      optional 3D mesh for rerun
    run_stats.json                 peak VRAM, inference time, scale factor

Design notes:
- Image-only inference: MapAnything estimates intrinsics itself, so we skip
  passing pinhole K. Fisheye → pinhole undistortion is already done upstream,
  which is what the model expects.
- memory_efficient_inference=True + minibatch_size=1 for low-VRAM GPUs.
"""

import argparse
import json
import os
import time
from pathlib import Path

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

import numpy as np
import torch

from mapanything.models import MapAnything
from mapanything.utils import image as mapanything_image
from mapanything.utils.image import load_images

# Patch: MapAnything's postprocess calls `.cpu().numpy()` at many sites on
# tensors that are bf16 because we forced bf16 model dtype. numpy cannot
# convert bf16 directly — auto-cast to fp32 at the boundary.
_orig_numpy = torch.Tensor.numpy


def _numpy_bf16_safe(self, *args, **kwargs):
    if self.dtype in (torch.bfloat16, torch.float16):
        return _orig_numpy(self.float(), *args, **kwargs)
    return _orig_numpy(self, *args, **kwargs)


torch.Tensor.numpy = _numpy_bf16_safe

# Patch: `recover_pinhole_intrinsics_from_ray_directions` calls
# `torch.linalg.solve`, which has no bf16 cusolver. Disable autocast and do
# the LU in fp32.
from mapanything.utils import geometry as _mg  # noqa: E402
from mapanything.utils import inference as _mi  # noqa: E402

_orig_recover = _mg.recover_pinhole_intrinsics_from_ray_directions


def _recover_pinhole_fp32(ray_directions, *args, **kwargs):
    with torch.autocast(device_type=ray_directions.device.type, enabled=False):
        if ray_directions.dtype in (torch.bfloat16, torch.float16):
            ray_directions = ray_directions.float()
        return _orig_recover(ray_directions, *args, **kwargs)


_mg.recover_pinhole_intrinsics_from_ray_directions = _recover_pinhole_fp32
_mi.recover_pinhole_intrinsics_from_ray_directions = _recover_pinhole_fp32

# Patch: `preprocess_input_views_for_inference` converts intrinsics → ray_dirs
# (and depth_z → depth_along_ray) in fp32 even when inputs are bf16, which
# then clashes with bf16 destinations inside `_encode_and_fuse_ray_dirs`.
# Cast any floating-point tensor outputs to match img dtype after preprocess.
_orig_preprocess = _mi.preprocess_input_views_for_inference


def _preprocess_views_dtype_safe(views, *a, **kw):
    processed = _orig_preprocess(views, *a, **kw)
    for pv in processed:
        img_dtype = pv["img"].dtype
        for k in (
            "ray_directions",
            "ray_directions_cam",
            "depth_along_ray",
            "cam_quats",
            "cam_trans",
        ):
            if k in pv and pv[k].dtype != img_dtype and pv[k].is_floating_point():
                pv[k] = pv[k].to(img_dtype)
    return processed


_mi.preprocess_input_views_for_inference = _preprocess_views_dtype_safe
# Also patch the attribute on MapAnything model module (which imports by name)
from mapanything.models.mapanything import model as _mm  # noqa: E402

_mm.preprocess_input_views_for_inference = _preprocess_views_dtype_safe
print(
    f"[patch] preprocess patched: {_mm.preprocess_input_views_for_inference is _preprocess_views_dtype_safe}"
)


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--input", required=True, help="Directory from extract_cam0_undist.py"
    )
    ap.add_argument("--output", required=True, help="Output directory (absolute)")
    ap.add_argument(
        "--apache",
        action="store_true",
        help="Use facebook/map-anything-apache (Apache 2.0) instead of CC-BY-NC",
    )
    ap.add_argument("--minibatch", type=int, default=1)
    ap.add_argument(
        "--no_glb", action="store_true", help="Skip GLB export (faster / less RAM)"
    )
    ap.add_argument("--max_frames", type=int, default=-1, help="-1 = all")
    ap.add_argument(
        "--pose_tum",
        default=None,
        help="Optional TUM trajectory to feed as camera_poses (MVS mode). "
        "Poses are interpolated to image timestamps. Pinhole intrinsics are "
        "also read from <input>/intrinsics.json and passed to the model.",
    )
    return ap.parse_args()


def load_timestamps(ts_path):
    ts = {}
    for line in Path(ts_path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        idx, t = line.split()
        ts[int(idx)] = float(t)
    return ts


def load_tum_trajectory(path):
    import numpy as np

    arr = np.loadtxt(path, comments="#")
    return arr[:, 0], arr[:, 1:4], arr[:, 4:8]  # t, xyz, quat(xyzw)


def quat_to_R(q):
    import numpy as np

    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def R_to_quat(R):
    """Rotation matrix → quaternion (xyzw). Shepperd's method, simple case."""
    import numpy as np

    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return np.array([qx, qy, qz, qw])


def interp_pose_at(ts, t_arr, xyz_arr, q_arr):
    """Linear/slerp-lite interpolation of TUM trajectory at times `ts`.
    Returns Nx4x4 cam2world matrices; None rows for out-of-range."""
    import numpy as np

    out = np.zeros((len(ts), 4, 4))
    valid = np.zeros(len(ts), dtype=bool)
    for i, t in enumerate(ts):
        if t < t_arr[0] or t > t_arr[-1]:
            continue
        j = int(np.searchsorted(t_arr, t))
        if j == 0:
            j = 1
        t0, t1 = t_arr[j - 1], t_arr[j]
        a = (t - t0) / max(t1 - t0, 1e-9)
        xyz = (1 - a) * xyz_arr[j - 1] + a * xyz_arr[j]
        q0, q1 = q_arr[j - 1], q_arr[j]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q = (1 - a) * q0 + a * q1
        q = q / np.linalg.norm(q)
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q)
        T[:3, 3] = xyz
        out[i] = T
        valid[i] = True
    return out, valid


def main():
    args = parse_args()
    in_dir = Path(args.input).expanduser().resolve()
    out_dir = Path(args.output).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    img_folder = in_dir / "images"
    timestamps = load_timestamps(in_dir / "timestamps.txt")
    img_files = sorted(img_folder.glob("*.jpg"))
    if args.max_frames > 0:
        img_files = img_files[: args.max_frames]
    print(f"[load] {len(img_files)} images from {img_folder}")

    views = load_images(
        folder_or_list=[str(p) for p in img_files],
        resolution_set=518,
        norm_type="dinov2",
        patch_size=14,
        verbose=False,
    )
    true_shape = views[0]["true_shape"][0]  # (H, W)
    new_H, new_W = int(true_shape[0]), int(true_shape[1])
    print(f"[load] resized to {new_W}x{new_H} (WxH), {len(views)} views")

    # --- Optional MVS mode: feed external poses + pinhole K, MapAnything
    # becomes pure dense reconstruction (no pose regression needed).
    mvs_mode = args.pose_tum is not None
    if mvs_mode:
        with open(in_dir / "intrinsics.json") as f:
            K_json = json.load(f)
        K_full = np.array(K_json["K"], dtype=np.float64)  # (3,3)
        W_full, H_full = int(K_json["W"]), int(K_json["H"])
        sx, sy = new_W / W_full, new_H / H_full
        K_scaled = K_full.copy()
        K_scaled[0, 0] *= sx
        K_scaled[1, 1] *= sy
        K_scaled[0, 2] *= sx
        K_scaled[1, 2] *= sy
        print(f"[mvs] scaled K to {new_W}x{new_H}:\n{K_scaled}")

        t_arr, xyz_arr, q_arr = load_tum_trajectory(args.pose_tum)
        ts_list = [timestamps[i] for i in range(len(views))]
        poses_np, valid = interp_pose_at(ts_list, t_arr, xyz_arr, q_arr)
        missing = (~valid).sum()
        if missing:
            print(f"[mvs] WARN: {missing} views outside {args.pose_tum} time range")
        # Attach K + pose to every view. Use (quats, trans) tuple form to avoid
        # the model's internal 4x4 → quat/trans decomposition (which mixes
        # fp32 intermediates with our bf16 and crashes).
        # All tensors bf16 to match the model dtype.
        K_t = torch.from_numpy(K_scaled).to(torch.bfloat16).unsqueeze(0)
        for i, v in enumerate(views):
            v["intrinsics"] = K_t.clone()
            T = poses_np[i]
            quat = R_to_quat(T[:3, :3])  # xyzw
            trans = T[:3, 3]
            v["camera_poses"] = (
                torch.from_numpy(quat).to(torch.bfloat16).unsqueeze(0),
                torch.from_numpy(trans).to(torch.bfloat16).unsqueeze(0),
            )
            v["is_metric_scale"] = torch.tensor([True])
        print(f"[mvs] attached K + pose to all {len(views)} views")

    device = "cuda" if torch.cuda.is_available() else "cpu"
    hf_id = "facebook/map-anything-apache" if args.apache else "facebook/map-anything"
    # Load on CPU → cast to bf16 → move to GPU.
    # ViT-g14 in fp32 is ~4.4GB; on an 8GB GPU with X server already eating
    # ~3GB, the fp32 `.to("cuda")` OOMs before inference even starts.
    print(f"[model] loading {hf_id} (cpu → bf16 → {device})")
    t0 = time.time()
    model = MapAnything.from_pretrained(hf_id)
    model = model.to(torch.bfloat16).to(device)
    model.eval()
    if device == "cuda":
        print(
            f"[model] loaded in {time.time() - t0:.1f}s  "
            f"VRAM after load={torch.cuda.memory_allocated() / 1024**3:.2f} GB"
        )
    else:
        print(f"[model] loaded in {time.time() - t0:.1f}s")

    if device == "cuda":
        torch.cuda.reset_peak_memory_stats()

    # Model weights are bf16 on GPU (to fit 8GB VRAM). Cast image tensors to
    # bf16 and also switch torch default dtype so MapAnything's internal
    # `torch.zeros`/`torch.ones` dummy-pose tensors default to bf16 too.
    for v in views:
        v["img"] = v["img"].to(torch.bfloat16)
    torch.set_default_dtype(torch.bfloat16)
    print("[infer] running...")
    t_inf = time.time()
    with torch.no_grad(), torch.autocast("cuda", dtype=torch.bfloat16):
        preds = model.infer(
            views,
            memory_efficient_inference=True,
            minibatch_size=args.minibatch,
            use_amp=False,
            apply_mask=True,
            mask_edges=True,
            apply_confidence_mask=False,
        )
    inf_dt = time.time() - t_inf
    peak_mem_gb = (
        torch.cuda.max_memory_allocated() / 1024**3 if device == "cuda" else 0.0
    )
    print(f"[infer] done in {inf_dt:.1f}s  peak VRAM={peak_mem_gb:.2f} GB")

    # --- Trajectory export (TUM) ---
    tum_lines = []
    scales = []
    for view_idx, pred in enumerate(preds):
        trans = pred["cam_trans"][0].float().cpu().numpy()  # (3,)
        quat = pred["cam_quats"][0].float().cpu().numpy()  # (4,) qx,qy,qz,qw
        scale = pred["metric_scaling_factor"][0].float().cpu().item()
        scales.append(scale)
        ts = timestamps.get(view_idx, float(view_idx))
        tum_lines.append(
            f"{ts:.9f} {trans[0]:.9f} {trans[1]:.9f} {trans[2]:.9f} "
            f"{quat[0]:.9f} {quat[1]:.9f} {quat[2]:.9f} {quat[3]:.9f}"
        )

    tum_path = out_dir / "trajectory_mapanything.tum"
    tum_path.write_text(
        "# TUM format: timestamp tx ty tz qx qy qz qw (cam2world, OpenCV)\n"
        + "\n".join(tum_lines)
        + "\n"
    )
    print(f"[out] wrote {tum_path} ({len(tum_lines)} poses)")
    print(
        f"[out] metric_scaling_factor mean={np.mean(scales):.4f}  "
        f"std={np.std(scales):.4f}"
    )

    # --- GLB export ---
    if not args.no_glb:
        from mapanything.utils.geometry import depthmap_to_world_frame
        from mapanything.utils.viz import predictions_to_glb

        world_pts_list, imgs_list, masks_list = [], [], []
        for pred in preds:
            # bf16 einsum in depthmap_to_world_frame isn't supported; cast fp32
            dm = pred["depth_z"][0].squeeze(-1).float()  # (H, W)
            K = pred["intrinsics"][0].float()  # (3, 3)
            pose = pred["camera_poses"][0].float()  # (4, 4)
            pts3d, valid = depthmap_to_world_frame(dm, K, pose)
            mask = pred["mask"][0].squeeze(-1).cpu().numpy().astype(bool)
            mask = mask & valid.cpu().numpy()
            world_pts_list.append(pts3d.cpu().numpy())
            imgs_list.append(pred["img_no_norm"][0].float().cpu().numpy())
            masks_list.append(mask)

        scene = predictions_to_glb(
            {
                "world_points": np.stack(world_pts_list, axis=0),
                "images": np.stack(imgs_list, axis=0),
                "final_masks": np.stack(masks_list, axis=0),
            },
            as_mesh=True,
        )
        glb_path = out_dir / "scene.glb"
        scene.export(str(glb_path))
        print(f"[out] wrote {glb_path}")

    stats = {
        "num_views": len(views),
        "resized_WxH": [int(true_shape[1]), int(true_shape[0])],
        "inference_seconds": round(inf_dt, 2),
        "peak_vram_gb": round(peak_mem_gb, 3),
        "metric_scale_mean": float(np.mean(scales)),
        "metric_scale_std": float(np.std(scales)),
        "model": hf_id,
        "minibatch": args.minibatch,
    }
    (out_dir / "run_stats.json").write_text(json.dumps(stats, indent=2))
    print(f"[out] run_stats.json: {stats}")


if __name__ == "__main__":
    main()
