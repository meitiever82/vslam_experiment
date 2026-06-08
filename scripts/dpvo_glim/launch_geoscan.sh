#!/bin/bash
# DPVO-GLIM 紧耦合集成联调 — GeoScan B1 bag (2026-04-20 夜写,2026-04-21 跑)
#
# 架构:DPVO mono VO 出 /dpvo/pose topic,glim_ext 的 libdpvo_frontend.so
# 订阅后把相邻 KF 的相对 pose 作 BetweenFactor 塞进 GLIM 的 factor graph。
#
# 要三个终端。这个脚本是参考用,不是一次性拉起。
#
# 先决条件(已在 2026-04-20 夜做完):
# - DPVO 的 dpvo_ros_node.py 已加 --pose_topic 选项 + PoseStamped publisher
# - glim_ext 已加模块 libdpvo_frontend.so + 已 colcon build 安装
# - handsfree/config_ros.json 的 extension_modules 里已加 libdpvo_frontend.so
#
# 已知的联调 TODO(明天要处理):
# 1. Bag 的 IMU topic 是 /handsfree/imu,GLIM handsfree config 期望 /imu → 用 bag play
#    --remap 改名,或者改 config。同理 /livox/lidar → /points,/left_camera/image → /image
# 2. /handsfree/imu 的 acc 是 g unit(|acc|≈1 静止),GLIM 要 m/s²,config 里已经有
#    "acc_scale": 9.80665,这部分应该已处理。verify 一下。
# 3. T_base_camera (DPVO 扩展里) 目前硬编码 Identity。要改为从 handsfree config 的
#    T_lidar_imu + T_lidar_camera 派生(按 orb_slam_frontend.cpp:62-64 的逻辑),
#    或先从 config_sensors_ext.json 读 T_lidar_camera 再算。如果 Identity 导致 err_r
#    > 0.3 rad,扩展会自动拒绝 factor(已加了 gate),就能看到日志警告 → 再回来填对。
# 4. DPVO 的相机标定(NEW_K 变量)现在用的是原始 fisheye K,未去畸变。去畸变后应该
#    仍用原 K(我们用 initUndistortRectifyMap 保持 K 不变)。verify 跟 DPVO 的
#    calib/geoscan.txt 一致。
# 5. Verify DPVO 的 BUFFER_SIZE=4096 够吃 4183 帧 bag。如果不够 → stride=2 或分段。
#
# 运行顺序(三终端):

# -- Terminal 1: DPVO node --
# source /opt/ros/humble/setup.bash
# cd /home/steve/vslam_ws/src/DPVO
# .venv/bin/python dpvo_ros_node.py \
#     --network dpvo.pth \
#     --pose_topic /dpvo/pose \
#     --save_trajectory /tmp/dpvo_geoscan_traj.tum

# -- Terminal 2: GLIM 带 dpvo_frontend 扩展 --
# source /opt/ros/humble/setup.bash
# source /home/steve/casbot_ws/install/setup.bash
# ros2 launch glim_ros2 glim_ros2.launch.py \
#     config_path:=/home/steve/casbot_ws/src/finder_lidar_mapping/glim/config/handsfree

# -- Terminal 3: Bag 回放(带 topic remap,让 GLIM handsfree config 找得到)--
# source /opt/ros/humble/setup.bash
# ros2 bag play ~/Documents/Datasets/geoscan/B1/2026-02-12-16-47-48 \
#     --clock --start-offset 10 \
#     --remap /handsfree/imu:=/imu \
#            /livox/lidar:=/points \
#            /left_camera/image:=/image

# 注意:DPVO 订阅的是 /left_camera/image,但 bag play remap 后这个名字变成 /image。
# 要么 DPVO 也订阅 /image(改 dpvo_ros_node.py:53),要么 bag 发双份(不 remap 但
# GLIM 额外加一个 /left_camera/image → /image 的 topic_tools relay)。
# 简洁方案:改 DPVO 订阅 /image,跟 GLIM 共用。
#
# 监控点:
# - rviz 里 odom 轨迹应连续
# - GLIM stdout 应看到 "injected N BetweenFactor(s) into smoother"
# - dpvo_frontend 日志打 "scale ready after X pairs" 后 current_scale 稳定
# - 如果 err_r > 0.3 rad 频繁 → T_base_camera 外参错(回来修 config)

echo "This is a reference script, not intended to be run directly."
echo "Copy-paste the three terminal commands above."
