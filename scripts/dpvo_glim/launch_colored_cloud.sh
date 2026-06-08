#!/bin/bash
# Realtime colored point cloud pipeline (Option A) for GeoScan B1.
#
# Architecture:
#   bag_replay_monotone.py  — strict monotone playback (ros2 bag play has bug)
#   ↓  /livox/imu /livox/lidar /left_camera/image
#   GLIM LIO  → /glim_ros/aligned_points_corrected (LiDAR in world)
#              /glim_ros/pose_corrected           (IMU pose in world)
#   colored_cloud_node.py   — project LiDAR pts into cam0 → /colored_cloud (RGB)
#   colored_cloud_to_ply.py — accumulate + auto-flush to PLY every 50 msgs
#
# Output: /home/steve/vslam_ws/runs/geoscan_colored/geoscan_b1_glim_colored.ply
#
# Open in MeshLab / CloudCompare / Rerun (drag and drop .ply).

set -e

BAG=/home/steve/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48
CFG=/home/steve/casbot_ws/src/finder_lidar_mapping/glim/config/geoscan_b1
OUT=/home/steve/vslam_ws/runs/geoscan_colored/geoscan_b1_glim_colored.ply
mkdir -p "$(dirname "$OUT")"

# Fresh state
ps -ef | grep python | grep -v grep | grep -v chrome | \
  grep -iE "glim_rosnode|colored_cloud|bag_replay|dpvo_ros" | \
  awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 2
ros2 daemon stop 2>&1 || true; sleep 1; ros2 daemon start; sleep 2

source /opt/ros/humble/setup.bash
source /home/steve/casbot_ws/install/setup.bash

# 1. GLIM LIO (baseline config without dpvo_frontend)
ros2 launch glim_ros glim_handsfree.launch.py \
  config_path:=$CFG \
  imu_topic:=/livox/imu points_topic:=/livox/lidar > /tmp/glim_colored.log 2>&1 &
sleep 10

# 2. Colored cloud node
python3 /home/steve/vslam_ws/src/vslam_experiment/scripts/dpvo_glim/colored_cloud_node.py \
  --config_dir $CFG > /tmp/colored.log 2>&1 &
sleep 3

# 3. PLY accumulator (auto-flush every 50 msgs)
python3 /home/steve/vslam_ws/src/vslam_experiment/scripts/dpvo_glim/colored_cloud_to_ply.py \
  --out "$OUT" --stride 3 > /tmp/ply_saver.log 2>&1 &
sleep 2

# 4. Monotone replayer (custom — ros2 bag play has out-of-order delivery bug)
python3 /home/steve/vslam_ws/src/vslam_experiment/scripts/dpvo_glim/bag_replay_monotone.py \
  --bag $BAG --clock --rate 1.0 --start-offset 10 \
  --topics /livox/imu /livox/lidar /left_camera/image > /tmp/bag_colored.log 2>&1 &

echo "Pipeline started. Monitor via:"
echo "  tail -f /tmp/ply_saver.log"
echo "  tail -f /tmp/colored.log"
echo "PLY accumulating at: $OUT"
echo ""
echo "To stop: ps -ef | grep python | grep -E 'glim|colored|bag_replay' | awk '{print \$2}' | xargs kill"
