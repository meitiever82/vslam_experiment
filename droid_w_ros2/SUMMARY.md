# droid_w_ros2 — work summary (2026-04-08)

ROS 2 wrapper for DROID-W (DROID-SLAM in the Wild). Subscribes to a monocular
image topic, buffers frames in memory, runs DROID-W in a background thread,
publishes the keyframe trajectory as `Path` / `PoseStamped` / `Odometry` /
TF, and dumps a TUM trajectory file on shutdown.

## Status

| step | state |
|---|---|
| install uv venv (py 3.12) | done |
| pytorch nightly cu130 (Blackwell) | done |
| `setup.py` `sm_120` gencode | done |
| build `thirdparty/lietorch` | done |
| build `thirdparty/diff-gaussian-rasterization-w-pose`, `simple-knn` | done |
| build `droid_backends` | done |
| `requirements.txt` + `mmcv` | done |
| **write `droid_w_ros2` ROS 2 package** | **done (this session)** |
| in-memory dataset adapter `BufferedRGB` | done |
| **test on a real ROS bag** | **TODO** |

`colcon build --packages-select droid_w_ros2 --symlink-install` passes;
`ros2 pkg executables droid_w_ros2` lists `droid_w_node`; module imports
cleanly under `source /opt/ros/jazzy/setup.bash && source install/setup.bash`.

## What lives where

### `/home/steve/vslam_ws/src/droid_w_ros2/`

```
package.xml                       # ament_python, deps: rclpy, sensor/nav/geometry/std_srvs, tf2_ros, cv_bridge
setup.py                          # entry_point: droid_w_node = droid_w_ros2.droid_w_node:main
setup.cfg
resource/droid_w_ros2
droid_w_ros2/__init__.py
droid_w_ros2/frame_buffer.py      # thread-safe FIFO: push / freeze / snapshot
droid_w_ros2/droid_w_node.py      # main node
launch/droid_w.launch.py          # loads config/droid_w_ros.yaml, supports params_file:= override
config/droid_w_ros.yaml           # all ROS parameters with absolute-path examples
```

### `/home/steve/Documents/GitHub/slam/DROID-W/` (modified)

- `src/utils/datasets.py` — added `BufferedRGB(RGB_NoPose)` (subclass to take
  the no-GT eval path); registered as `dataset_dict['ros'] = BufferedRGB`.
  Frames are passed as `[(timestamp_sec, np.ndarray BGR uint8)]`. Skips
  `RGB_NoPose.__init__` (which scans disk) and calls `BaseDataset.__init__`
  directly.
- `src/slam.py` — `from src.gui import gui_utils, slam_gui` was moved out of
  the top-level imports and into `if self.cfg['gui']:` inside `run()`. Lets
  the headless ROS path import SLAM without `glfw` / `PyQt5` / `open3d`.

## Node behavior

1. Subscribes to `image_topic` (`sensor_msgs/Image`, optionally
   `CompressedImage` if `compressed=true`). cv_bridge → BGR → `FrameBuffer`.
   Honors `frame_skip` (use every Nth) and `max_frames` (cap, 0 = unbounded).
2. SLAM is launched in two ways:
   - **manual**: `ros2 service call /droid_w_node/start std_srvs/srv/Trigger`
   - **auto**: as soon as `auto_start_after_n` frames are buffered (>0).
3. Launch sequence (in a Python `threading.Thread`, daemon=True):
   - load DROID-W YAML, set `cfg['dataset']='ros'`, override `cfg['scene']`
     and `cfg['data']['output']`, force `gui=False`, `droidvis=False`,
     `mapping.final_refine_iters=3000` (in fast_mode), call `setup_seed`.
   - `BufferedRGB(cfg, frames)` → `SLAM(cfg, stream)` → `slam.run()`.
   - `slam.run()` internally calls `mp.set_start_method('spawn', force=True)`
     and spawns the tracking subprocess; this is fine from a Python thread
     since rclpy doesn't use multiprocessing.
4. A wall timer (`publish_rate_hz`, default 10 Hz) reads
   `slam.video.counter.value` and `slam.video.poses[:n]` from the shared
   memory tensors, inverts each w2c → c2w, looks up the wall-clock time via
   `BufferedRGB.image_timestamps[int(slam.video.timestamp[k])]`, and
   publishes:
     - `~/kf_path` `nav_msgs/Path`
     - `~/kf_pose` `geometry_msgs/PoseStamped` (latest only)
     - `~/odom` `nav_msgs/Odometry` (latest only)
     - TF: `world_frame_id` → `camera_frame_id`
5. On shutdown, if `output_traj_path` is set, the latest published trajectory
   is dumped as TUM format (`ts tx ty tz qx qy qz qw`).

## Critical detail: timestamp semantics

`slam.video.timestamp[k]` is **the source frame index in the dataset**, NOT
a wall-clock time. Confirmed by `src/utils/eval_traj.py:27`
(`int(video_timestamps[i])` is used as an index into `stream.poses`).

The publish tick must therefore look up
`BufferedRGB.image_timestamps[int(slam.video.timestamp[k])]` to recover the
real ROS time. This is cached as `self._stream_timestamps` at SLAM-launch
time. Don't regress this — using `slam.video.timestamp` directly as a
header.stamp gives nonsense values like `1970-01-01 00:00:23`.

## Coordinate convention

`DepthVideo.poses` stores SE3 elements as `[tx, ty, tz, qx, qy, qz, qw]`
with the **lietorch** convention. Per the comment in `depth_video.py:56`,
they are **world-to-camera**. To publish as TF / Path, the node calls
`_invert_w2c`:
- `q_inv = (-qx, -qy, -qz, qw)` (unit quaternion conjugate)
- `t_inv = quat_rotate(q_inv, -t)` = `-Rᵀ t`

## Issues we already fixed in this session

| # | bug | fix |
|---|---|---|
| 1 | publish tick used `video.timestamp[k]` (= source frame index) directly as ROS time → garbage `header.stamp` | cache `stream.image_timestamps`; `sec = stream_ts[int(kf_idx[i])]` |
| 2 | `cfg['data']['output']` was a relative path (`./Outputs/Bonn`); SLAM would dump under whatever cwd `ros2 launch` had | added ROS params `output_dir` and `scene`, force-overridden in `_launch_slam` |
| 3 | `cfg.setdefault('mapping',{}).setdefault('final_refine_iters', 3000)` was a no-op (the YAML already has 20000) | use direct assignment, like `run.py` does |
| 4 | did not call `setup_seed` so reproducibility was lost vs `run.py` | added `setup_seed(cfg.get('setup_seed', 43))` |
| 5 | a SLAM-launch failure left `_slam_thread is None`, so `auto_start_after_n` would re-fire on every subsequent frame | added a sticky `self._slam_started` flag, set up-front in `_launch_slam` |
| 6 | the python module dir got renamed to `src/` (breaks `droid_w_ros2.droid_w_node` import) | `mv src/ → droid_w_ros2/` |

Quaternion / rotation math was hand-checked and is correct.

## How to run (after restart)

```bash
# 1. Source ROS + workspace
source /opt/ros/jazzy/setup.bash
source /home/steve/vslam_ws/install/setup.bash

# 2. (If you change anything in the package, rebuild — symlink-install means
#    Python edits are picked up live.)
cd /home/steve/vslam_ws
colcon build --packages-select droid_w_ros2 --symlink-install

# 3. Edit config/droid_w_ros.yaml — set:
#      droid_root, droid_config, image_topic, output_dir, output_traj_path
#    The default droid_config points at bonn_dynamic.yaml which has Bonn's
#    intrinsics; for any other camera you must point at a YAML whose `cam:`
#    block matches your sensor.

# 4. Launch
ros2 launch droid_w_ros2 droid_w.launch.py

# 5. Replay your bag in another terminal
ros2 bag play /path/to/bag

# 6. Trigger SLAM (or set auto_start_after_n > 0 in the yaml)
ros2 service call /droid_w_node/start std_srvs/srv/Trigger

# 7. Watch
ros2 topic echo /droid_w_node/kf_pose
ros2 topic echo /droid_w_node/odom
ros2 run rviz2 rviz2     # add Path on /droid_w_node/kf_path, fixed frame droid_world
```

## ROS parameters (config/droid_w_ros.yaml)

| param | default | meaning |
|---|---|---|
| `droid_root` | (must set) | abs path to DROID-W repo (added to `sys.path`) |
| `droid_config` | (must set) | abs path to a DROID-W YAML config |
| `image_topic` | `/camera/color/image_raw` | input topic |
| `compressed` | `false` | use `CompressedImage` |
| `frame_skip` | `1` | take every Nth incoming frame |
| `max_frames` | `1500` | cap on buffered frames (0 = unbounded) |
| `auto_start_after_n` | `0` | >0 launches SLAM after N frames (else manual via `~/start`) |
| `world_frame_id` | `droid_world` | TF parent / Path frame |
| `camera_frame_id` | `droid_camera` | TF child |
| `publish_rate_hz` | `10.0` | trajectory republish rate |
| `output_dir` | `''` | overrides `cfg['data']['output']` (set this!) |
| `scene` | `ros_session` | overrides `cfg['scene']` |
| `output_traj_path` | `''` | TUM dump path on shutdown (`''` = skip) |

## Known limitations / TODOs

- **Not actually streaming**: SLAM runs once, on a frozen snapshot of the
  buffer. The "real-time" aspect is only frame collection from ROS, not
  online SLAM. To do online tracking, the loop in
  `src/tracker.py:Tracker.run` (`for i in range(len(stream))`) would need to
  be patched to `while`-style with a generator-like stream.
- **CameraInfo not honored**: intrinsics come from the DROID-W YAML config.
  Could optionally subscribe to `/camera/camera_info` and override `cfg['cam']`
  before constructing BufferedRGB.
- **Pickle cost across spawn**: in spawn-mode, the BufferedRGB (with all the
  frames as numpy arrays) gets pickled into the tracker child process. For a
  900-frame 640×480 buffer that's ~250 MB; tolerable but not free. Cap with
  `max_frames` if you hit pressure.
- **No PointCloud2 of keyframes** (was mentioned in the v0.1.0 package.xml
  description). Not implemented; would require unprojecting `video.disps_up`
  to a point cloud per keyframe.
- **Concurrent shared-tensor reads**: the publish tick `.clone()`s
  `slam.video.poses[:n]` from the rclpy thread while a child process is
  writing. Worst case is one tick of garbage that the next tick fixes — has
  a `try/except` around it.

## Where the previous session got interrupted

This file marks the **end of "write ROS 2 package"** plus all the bug fixes
caught in code review. The remaining checklist item is:

- [ ] **test on a real bag** — needs a bag path / camera topic from the user.

When resuming: source the workspace (step 1 above), edit
`config/droid_w_ros.yaml` for your camera, then go to step 4.
