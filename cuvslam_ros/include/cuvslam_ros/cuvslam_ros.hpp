// Copyright (c) 2026, Steve. All rights reserved.
// ROS2 wrapper for NVIDIA cuVSLAM

#pragma once

#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

// cuVSLAM
#include <cuvslam/cuvslam2.h>

namespace cuvslam_ros {

class CuvslamROS : public rclcpp::Node {
public:
  explicit CuvslamROS(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CuvslamROS();

private:
  // ---- Initialization ----
  void declare_parameters();
  void setup_camera_rig();
  void setup_tracker();
  void setup_publishers();
  void setup_subscribers();

  // ---- Callbacks ----
  void stereo_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);
  void rgbd_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
  void mono_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void drain_imu_until(int64_t image_ts_ns);
  void log_slam_status();
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

  // ---- Publishing ----
  void publish_odometry(const cuvslam::PoseEstimate& estimate, const rclcpp::Time& stamp);
  void publish_landmarks(const rclcpp::Time& stamp);

  // ---- Coordinate conversion ----
  // cuVSLAM uses OpenCV convention (x-right, y-down, z-forward)
  // ROS uses (x-forward, y-left, z-up)
  static geometry_msgs::msg::Pose opencv_to_ros_pose(const cuvslam::Pose& cv_pose);

  // ---- Convert ROS Time to nanoseconds ----
  static int64_t stamp_to_ns(const rclcpp::Time& stamp);

  // ---- Parameters ----
  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string depth_image_topic_;
  std::string imu_topic_;
  std::string camera_info_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string camera_frame_id_;

  // Camera intrinsics (from params or CameraInfo)
  int image_width_;
  int image_height_;
  double fx_, fy_, cx_, cy_;
  std::string distortion_model_;  // "pinhole", "brown", "fisheye"
  std::vector<double> distortion_coeffs_;

  // Right camera intrinsics (if different from left; 0 = same as left)
  double right_fx_, right_fy_, right_cx_, right_cy_;
  std::vector<double> right_distortion_coeffs_;

  // Stereo extrinsics: rig_from_cam1 (quaternion + translation)
  double rig_from_cam1_qx_, rig_from_cam1_qy_, rig_from_cam1_qz_, rig_from_cam1_qw_;
  double rig_from_cam1_tx_, rig_from_cam1_ty_, rig_from_cam1_tz_;

  // IMU extrinsics: rig_from_imu (quaternion + translation)
  double rig_from_imu_qx_, rig_from_imu_qy_, rig_from_imu_qz_, rig_from_imu_qw_;
  double rig_from_imu_tx_, rig_from_imu_ty_, rig_from_imu_tz_;

  // Odometry mode
  std::string odometry_mode_;  // "stereo", "inertial", "rgbd"
  bool enable_slam_;
  bool enable_landmarks_;
  int verbosity_;
  float depth_scale_factor_;
  std::string trajectory_output_path_;
  std::ofstream trajectory_file_;

  bool rectify_fisheye_ = false;
  cv::Mat rect_map1_left_, rect_map2_left_;
  cv::Mat rect_map1_right_, rect_map2_right_;
  void compute_fisheye_rectification();

  // ---- cuVSLAM objects ----
  std::unique_ptr<cuvslam::Rig> rig_;
  std::unique_ptr<cuvslam::Odometry> odometry_;
  std::unique_ptr<cuvslam::Slam> slam_;
  bool tracker_initialized_ = false;
  bool camera_info_received_ = false;

  // ---- ROS2 subscribers ----
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<Synchronizer> stereo_sync_;
  std::shared_ptr<Synchronizer> rgbd_sync_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mono_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Callback groups for concurrent processing
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;

  // ---- ROS2 publishers ----
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ---- State ----
  nav_msgs::msg::Path path_msg_;
  std::mutex tracker_mutex_;
  uint32_t frame_count_ = 0;

  // IMU buffer — register into cuVSLAM only before Track() so timestamps stay monotonic
  std::mutex imu_buffer_mutex_;
  std::deque<cuvslam::ImuMeasurement> imu_buffer_;
  int64_t last_registered_imu_ts_ = 0;

  // Loop-closure event tracking for logging
  size_t last_lc_pose_count_ = 0;
};

}  // namespace cuvslam_ros
