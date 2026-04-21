// Copyright (c) 2026, Steve. All rights reserved.
// ROS2 wrapper for NVIDIA cuVSLAM

#include "cuvslam_ros/cuvslam_ros.hpp"

#include <iomanip>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace cuvslam_ros {

CuvslamROS::CuvslamROS(const rclcpp::NodeOptions& options)
    : Node("cuvslam_ros", options) {
  RCLCPP_INFO(get_logger(), "Initializing cuVSLAM ROS2 node");

  // Print cuVSLAM version
  int32_t major, minor, patch;
  auto version = cuvslam::GetVersion(&major, &minor, &patch);
  RCLCPP_INFO(get_logger(), "cuVSLAM version: %s", std::string(version).c_str());

  declare_parameters();

  // Set cuVSLAM verbosity
  cuvslam::SetVerbosity(verbosity_);

  // Warm up GPU
  try {
    cuvslam::WarmUpGPU();
    RCLCPP_INFO(get_logger(), "GPU warmed up successfully");
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(get_logger(), "GPU warm-up failed: %s", e.what());
    return;
  }

  setup_publishers();

  // If camera intrinsics are provided via params, initialize tracker directly
  if (image_width_ > 0 && image_height_ > 0 && fx_ > 0) {
    setup_camera_rig();
    setup_tracker();
    tracker_initialized_ = true;
    camera_info_received_ = true;
  }

  setup_subscribers();

  RCLCPP_INFO(get_logger(), "cuVSLAM ROS2 node ready. Waiting for images...");
}

CuvslamROS::~CuvslamROS() {
  if (trajectory_file_.is_open()) {
    trajectory_file_.flush();
    trajectory_file_.close();
    RCLCPP_INFO(get_logger(), "Closed TUM trajectory file: %s", trajectory_output_path_.c_str());
  }
  RCLCPP_INFO(get_logger(), "Shutting down cuVSLAM ROS2 node. Processed %u frames.", frame_count_);
}

void CuvslamROS::declare_parameters() {
  // Topics
  left_image_topic_ = declare_parameter("left_image_topic", "/camera/left/image_raw");
  right_image_topic_ = declare_parameter("right_image_topic", "/camera/right/image_raw");
  depth_image_topic_ = declare_parameter("depth_image_topic", "");
  imu_topic_ = declare_parameter("imu_topic", "");
  camera_info_topic_ = declare_parameter("camera_info_topic", "");

  // Frame IDs
  odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  camera_frame_id_ = declare_parameter("camera_frame_id", "camera_link");

  // Camera intrinsics (can be 0 to wait for CameraInfo)
  image_width_ = declare_parameter("image_width", 0);
  image_height_ = declare_parameter("image_height", 0);
  fx_ = declare_parameter("fx", 0.0);
  fy_ = declare_parameter("fy", 0.0);
  cx_ = declare_parameter("cx", 0.0);
  cy_ = declare_parameter("cy", 0.0);
  distortion_model_ = declare_parameter("distortion_model", "pinhole");
  distortion_coeffs_ = declare_parameter("distortion_coeffs", std::vector<double>{});

  // Right camera intrinsics (0 = same as left)
  right_fx_ = declare_parameter("right_fx", 0.0);
  right_fy_ = declare_parameter("right_fy", 0.0);
  right_cx_ = declare_parameter("right_cx", 0.0);
  right_cy_ = declare_parameter("right_cy", 0.0);
  right_distortion_coeffs_ = declare_parameter("right_distortion_coeffs", std::vector<double>{});

  // Stereo extrinsics: rig_from_cam1 (default: +12cm baseline along x)
  rig_from_cam1_qx_ = declare_parameter("rig_from_cam1_qx", 0.0);
  rig_from_cam1_qy_ = declare_parameter("rig_from_cam1_qy", 0.0);
  rig_from_cam1_qz_ = declare_parameter("rig_from_cam1_qz", 0.0);
  rig_from_cam1_qw_ = declare_parameter("rig_from_cam1_qw", 1.0);
  rig_from_cam1_tx_ = declare_parameter("rig_from_cam1_tx", 0.12);
  rig_from_cam1_ty_ = declare_parameter("rig_from_cam1_ty", 0.0);
  rig_from_cam1_tz_ = declare_parameter("rig_from_cam1_tz", 0.0);

  // IMU extrinsics: rig_from_imu (default: identity; overwrite from Kalibr imu-cam calib)
  rig_from_imu_qx_ = declare_parameter("rig_from_imu_qx", 0.0);
  rig_from_imu_qy_ = declare_parameter("rig_from_imu_qy", 0.0);
  rig_from_imu_qz_ = declare_parameter("rig_from_imu_qz", 0.0);
  rig_from_imu_qw_ = declare_parameter("rig_from_imu_qw", 1.0);
  rig_from_imu_tx_ = declare_parameter("rig_from_imu_tx", 0.0);
  rig_from_imu_ty_ = declare_parameter("rig_from_imu_ty", 0.0);
  rig_from_imu_tz_ = declare_parameter("rig_from_imu_tz", 0.0);

  // Tracker config
  odometry_mode_ = declare_parameter("odometry_mode", "stereo");  // "stereo", "inertial", "rgbd", "mono"
  enable_slam_ = declare_parameter("enable_slam", false);
  enable_landmarks_ = declare_parameter("enable_landmarks", false);
  verbosity_ = declare_parameter("verbosity", 1);
  depth_scale_factor_ = declare_parameter("depth_scale_factor", 1000.0f);  // RealSense: mm->m
  trajectory_output_path_ = declare_parameter("trajectory_output_path", "");
  rectify_fisheye_ = declare_parameter("rectify_fisheye", false);
  if (!trajectory_output_path_.empty()) {
    trajectory_file_.open(trajectory_output_path_);
    if (trajectory_file_.is_open()) {
      trajectory_file_ << "# TUM trajectory from cuvslam_ros\n# timestamp tx ty tz qx qy qz qw\n";
      trajectory_file_ << std::fixed;
      RCLCPP_INFO(get_logger(), "Writing TUM trajectory to: %s", trajectory_output_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open trajectory_output_path: %s", trajectory_output_path_.c_str());
    }
  }
}

void CuvslamROS::setup_camera_rig() {
  rig_ = std::make_unique<cuvslam::Rig>();

  // Helper to parse distortion model string
  auto parse_distortion = [](const std::string& model, const std::vector<double>& coeffs) {
    cuvslam::Distortion dist;
    if (model == "fisheye") {
      dist.model = cuvslam::Distortion::Model::Fisheye;
    } else if (model == "brown") {
      dist.model = cuvslam::Distortion::Model::Brown;
    } else if (model == "polynomial") {
      dist.model = cuvslam::Distortion::Model::Polynomial;
    } else {
      dist.model = cuvslam::Distortion::Model::Pinhole;
    }
    for (auto c : coeffs) {
      dist.parameters.push_back(static_cast<float>(c));
    }
    return dist;
  };

  if (odometry_mode_ == "rgbd" || odometry_mode_ == "mono") {
    // RGBD / Mono mode: single camera (left)
    rig_->cameras.resize(1);
    auto& cam0 = rig_->cameras[0];
    cam0.size = {image_width_, image_height_};
    cam0.focal = {static_cast<float>(fx_), static_cast<float>(fy_)};
    cam0.principal = {static_cast<float>(cx_), static_cast<float>(cy_)};
    cam0.rig_from_camera.rotation = {0, 0, 0, 1};
    cam0.rig_from_camera.translation = {0, 0, 0};
    cam0.distortion = parse_distortion(distortion_model_, distortion_coeffs_);

    RCLCPP_INFO(get_logger(), "%s camera: %dx%d, fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
      odometry_mode_.c_str(), image_width_, image_height_, fx_, fy_, cx_, cy_);
  } else {
    // Stereo / Inertial mode: two cameras
    rig_->cameras.resize(2);

    auto& cam0 = rig_->cameras[0];
    cam0.size = {image_width_, image_height_};
    cam0.focal = {static_cast<float>(fx_), static_cast<float>(fy_)};
    cam0.principal = {static_cast<float>(cx_), static_cast<float>(cy_)};
    cam0.rig_from_camera.rotation = {0, 0, 0, 1};
    cam0.rig_from_camera.translation = {0, 0, 0};
    cam0.distortion = parse_distortion(distortion_model_, distortion_coeffs_);

    auto& cam1 = rig_->cameras[1];
    cam1.size = {image_width_, image_height_};

    float r_fx = right_fx_ > 0 ? static_cast<float>(right_fx_) : static_cast<float>(fx_);
    float r_fy = right_fy_ > 0 ? static_cast<float>(right_fy_) : static_cast<float>(fy_);
    float r_cx = right_cx_ > 0 ? static_cast<float>(right_cx_) : static_cast<float>(cx_);
    float r_cy = right_cy_ > 0 ? static_cast<float>(right_cy_) : static_cast<float>(cy_);
    cam1.focal = {r_fx, r_fy};
    cam1.principal = {r_cx, r_cy};

    cam1.rig_from_camera.rotation = {
      static_cast<float>(rig_from_cam1_qx_),
      static_cast<float>(rig_from_cam1_qy_),
      static_cast<float>(rig_from_cam1_qz_),
      static_cast<float>(rig_from_cam1_qw_)
    };
    cam1.rig_from_camera.translation = {
      static_cast<float>(rig_from_cam1_tx_),
      static_cast<float>(rig_from_cam1_ty_),
      static_cast<float>(rig_from_cam1_tz_)
    };

    auto& r_dist_coeffs = right_distortion_coeffs_.empty() ? distortion_coeffs_ : right_distortion_coeffs_;
    cam1.distortion = parse_distortion(distortion_model_, r_dist_coeffs);

    RCLCPP_INFO(get_logger(), "Stereo rig: %dx%d", image_width_, image_height_);
    RCLCPP_INFO(get_logger(), "  cam0: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx_, fy_, cx_, cy_);
    RCLCPP_INFO(get_logger(), "  cam1: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", r_fx, r_fy, r_cx, r_cy);
    RCLCPP_INFO(get_logger(), "  baseline: [%.4f, %.4f, %.4f]",
      rig_from_cam1_tx_, rig_from_cam1_ty_, rig_from_cam1_tz_);

    if (rectify_fisheye_ && distortion_model_ == "fisheye") {
      compute_fisheye_rectification();
    }
  }

  // IMU (if inertial mode)
  if (odometry_mode_ == "inertial" && !imu_topic_.empty()) {
    rig_->imus.resize(1);
    auto& imu = rig_->imus[0];
    // Normalize quaternion: cuVSLAM checks |det(R) - 1| < 1e-6 and will reject
    // slightly off-norm inputs (e.g. 6-digit truncated values).
    double qn = std::sqrt(rig_from_imu_qx_*rig_from_imu_qx_ +
                          rig_from_imu_qy_*rig_from_imu_qy_ +
                          rig_from_imu_qz_*rig_from_imu_qz_ +
                          rig_from_imu_qw_*rig_from_imu_qw_);
    if (qn < 1e-9) qn = 1.0;
    imu.rig_from_imu.rotation = {
      static_cast<float>(rig_from_imu_qx_ / qn),
      static_cast<float>(rig_from_imu_qy_ / qn),
      static_cast<float>(rig_from_imu_qz_ / qn),
      static_cast<float>(rig_from_imu_qw_ / qn)
    };
    imu.rig_from_imu.translation = {
      static_cast<float>(rig_from_imu_tx_),
      static_cast<float>(rig_from_imu_ty_),
      static_cast<float>(rig_from_imu_tz_)
    };
    RCLCPP_INFO(get_logger(),
      "IMU extrinsics rig_from_imu: t=(%.4f, %.4f, %.4f), q=(%.4f, %.4f, %.4f, %.4f)",
      rig_from_imu_tx_, rig_from_imu_ty_, rig_from_imu_tz_,
      rig_from_imu_qx_, rig_from_imu_qy_, rig_from_imu_qz_, rig_from_imu_qw_);
    imu.gyroscope_noise_density = declare_parameter("gyroscope_noise_density", 1.6968e-4f);
    imu.gyroscope_random_walk = declare_parameter("gyroscope_random_walk", 1.9393e-5f);
    imu.accelerometer_noise_density = declare_parameter("accelerometer_noise_density", 2.0e-3f);
    imu.accelerometer_random_walk = declare_parameter("accelerometer_random_walk", 3.0e-3f);
    imu.frequency = declare_parameter("imu_frequency", 200.0f);
  }
}

void CuvslamROS::compute_fisheye_rectification() {
  cv::Mat K1 = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
  cv::Mat D1 = (cv::Mat_<double>(4, 1) << distortion_coeffs_[0], distortion_coeffs_[1],
                distortion_coeffs_[2], distortion_coeffs_[3]);
  double rfx = right_fx_ > 0 ? right_fx_ : fx_;
  double rfy = right_fy_ > 0 ? right_fy_ : fy_;
  double rcx = right_cx_ > 0 ? right_cx_ : cx_;
  double rcy = right_cy_ > 0 ? right_cy_ : cy_;
  cv::Mat K2 = (cv::Mat_<double>(3, 3) << rfx, 0, rcx, 0, rfy, rcy, 0, 0, 1);
  const auto& dr = right_distortion_coeffs_.empty() ? distortion_coeffs_ : right_distortion_coeffs_;
  cv::Mat D2 = (cv::Mat_<double>(4, 1) << dr[0], dr[1], dr[2], dr[3]);

  // We have rig_from_cam1 = T_c0_c1 (point in cam1 -> point in cam0).
  // OpenCV stereoRectify wants R, T such that p_cam2 = R * p_cam1 + T,
  // where (cam1, cam2) in the API == (left, right) here. So:
  //   R_api = R_c1_c0 = R_c0_c1^T
  //   T_api = -R_c1_c0 * t_c0_c1
  Eigen::Quaterniond q_c0_c1(rig_from_cam1_qw_, rig_from_cam1_qx_,
                             rig_from_cam1_qy_, rig_from_cam1_qz_);
  q_c0_c1.normalize();
  Eigen::Matrix3d R_c0_c1 = q_c0_c1.toRotationMatrix();
  Eigen::Vector3d t_c0_c1(rig_from_cam1_tx_, rig_from_cam1_ty_, rig_from_cam1_tz_);
  Eigen::Matrix3d R_c1_c0 = R_c0_c1.transpose();
  Eigen::Vector3d t_c1_c0 = -R_c1_c0 * t_c0_c1;

  cv::Mat R_cv(3, 3, CV_64F);
  cv::Mat T_cv(3, 1, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) R_cv.at<double>(i, j) = R_c1_c0(i, j);
    T_cv.at<double>(i, 0) = t_c1_c0(i);
  }

  // Per-camera undistort to pinhole at the SAME intrinsics (no stereo rectification).
  // OpenCV's fisheye::stereoRectify degenerates on this ~108° HFOV input. We instead
  // undistort each fisheye image to a pinhole at the original K (R = identity), which
  // removes the barrel distortion that breaks cuvslam's internal "pinhole+undistort"
  // approach. The original stereo extrinsics (rig_from_cam1) stay in the cuvslam Rig.
  (void)R_c1_c0; (void)t_c1_c0; (void)R_cv; (void)T_cv;  // unused for per-cam undistort
  cv::Size image_size(image_width_, image_height_);
  cv::fisheye::initUndistortRectifyMap(K1, D1, cv::Mat::eye(3, 3, CV_64F), K1,
                                       image_size, CV_16SC2,
                                       rect_map1_left_, rect_map2_left_);
  cv::fisheye::initUndistortRectifyMap(K2, D2, cv::Mat::eye(3, 3, CV_64F), K2,
                                       image_size, CV_16SC2,
                                       rect_map1_right_, rect_map2_right_);

  // Override the cuvslam Rig to use pinhole (same K, no distortion coeffs) but keep
  // the original stereo extrinsics (already set above by the stereo branch).
  rig_->cameras[0].distortion.model = cuvslam::Distortion::Model::Pinhole;
  rig_->cameras[0].distortion.parameters.clear();
  rig_->cameras[1].distortion.model = cuvslam::Distortion::Model::Pinhole;
  rig_->cameras[1].distortion.parameters.clear();

  RCLCPP_INFO(get_logger(),
    "Fisheye undistort ON (per-camera, no rectification). cuvslam now sees pinhole stereo at original K.");
}

void CuvslamROS::setup_tracker() {
  // Configure odometry
  cuvslam::Odometry::Config config;
  config.use_gpu = true;
  config.async_sba = true;
  config.use_motion_model = true;
  config.use_denoising = false;

  if (odometry_mode_ == "inertial") {
    config.odometry_mode = cuvslam::Odometry::OdometryMode::Inertial;
  } else if (odometry_mode_ == "rgbd") {
    config.odometry_mode = cuvslam::Odometry::OdometryMode::RGBD;
    config.rgbd_settings.depth_scale_factor = depth_scale_factor_;
    config.rgbd_settings.depth_camera_id = 0;
  } else if (odometry_mode_ == "mono") {
    config.odometry_mode = cuvslam::Odometry::OdometryMode::Mono;
  } else {
    config.odometry_mode = cuvslam::Odometry::OdometryMode::Multicamera;
    config.multicam_mode = cuvslam::Odometry::MulticameraMode::Precision;
  }

  if (enable_slam_ || enable_landmarks_) {
    config.enable_observations_export = true;
    config.enable_landmarks_export = true;
  }

  try {
    odometry_ = std::make_unique<cuvslam::Odometry>(*rig_, config);
    RCLCPP_INFO(get_logger(), "Odometry tracker created (mode: %s)", odometry_mode_.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to create odometry tracker: %s", e.what());
    return;
  }

  // Optional SLAM
  if (enable_slam_) {
    cuvslam::Slam::Config slam_config;
    slam_config.use_gpu = true;
    slam_config.sync_mode = false;
    slam_config.enable_reading_internals = true;
    slam_config.max_map_size = 300;

    try {
      auto primary_cameras = odometry_->GetPrimaryCameras();
      slam_ = std::make_unique<cuvslam::Slam>(*rig_, primary_cameras, slam_config);
      // Enable reading map/current-frame landmarks for visualization
      slam_->EnableReadingData(cuvslam::Slam::DataLayer::Map, 10000);
      slam_->EnableReadingData(cuvslam::Slam::DataLayer::Landmarks, 1000);
      RCLCPP_INFO(get_logger(), "SLAM enabled (landmark data layers active)");
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to create SLAM: %s. Running odometry only.", e.what());
    }
  }
}

void CuvslamROS::setup_publishers() {
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  if (enable_landmarks_) {
    landmark_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/landmarks", 10);
  }

  path_msg_.header.frame_id = odom_frame_id_;
}

void CuvslamROS::setup_subscribers() {
  // Callback groups for concurrent IMU + image processing
  imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Use system default QoS (RELIABLE) for compatibility with rosbag and most publishers
  rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_default;
  auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(sensor_qos_profile));

  if (odometry_mode_ == "mono") {
    // Mono: subscribe to left camera only, no sync
    rclcpp::SubscriptionOptions img_opts;
    img_opts.callback_group = image_callback_group_;
    mono_sub_ = create_subscription<sensor_msgs::msg::Image>(
      left_image_topic_, sensor_qos,
      std::bind(&CuvslamROS::mono_callback, this, std::placeholders::_1),
      img_opts);
    RCLCPP_INFO(get_logger(), "Mono subscriber on: %s", left_image_topic_.c_str());
  } else if (odometry_mode_ == "rgbd") {
    // RGBD: sync RGB + Depth
    left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, left_image_topic_, sensor_qos_profile);
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, depth_image_topic_, sensor_qos_profile);

    rgbd_sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), *left_sub_, *depth_sub_);
    rgbd_sync_->registerCallback(
      std::bind(&CuvslamROS::rgbd_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "RGBD subscribers on: %s, %s",
      left_image_topic_.c_str(), depth_image_topic_.c_str());
  } else {
    // Stereo: sync Left + Right
    left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, left_image_topic_, sensor_qos_profile);
    right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, right_image_topic_, sensor_qos_profile);

    stereo_sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), *left_sub_, *right_sub_);
    stereo_sync_->registerCallback(
      std::bind(&CuvslamROS::stereo_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Stereo subscribers on: %s, %s",
      left_image_topic_.c_str(), right_image_topic_.c_str());
  }

  // IMU subscriber (if inertial mode)
  if (!imu_topic_.empty()) {
    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = imu_callback_group_;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&CuvslamROS::imu_callback, this, std::placeholders::_1),
      imu_opts);
    RCLCPP_INFO(get_logger(), "IMU subscriber on: %s", imu_topic_.c_str());
  }

  // CameraInfo subscriber (if no intrinsics from params)
  if (!camera_info_received_ && !camera_info_topic_.empty()) {
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&CuvslamROS::camera_info_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Waiting for CameraInfo on: %s", camera_info_topic_.c_str());
  }
}

// ============================================================================
// Callbacks
// ============================================================================

void CuvslamROS::stereo_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& right_msg) {

  if (!tracker_initialized_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Tracker not initialized. Waiting for camera info...");
    return;
  }

  std::lock_guard<std::mutex> lock(tracker_mutex_);

  // Convert ROS images to grayscale via cv_bridge
  cv::Mat left_gray, right_gray;
  try {
    auto left_cv = cv_bridge::toCvShare(left_msg, "mono8");
    auto right_cv = cv_bridge::toCvShare(right_msg, "mono8");
    left_gray = left_cv->image;
    right_gray = right_cv->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
    return;
  }

  cv::Mat left_rect, right_rect;
  if (!rect_map1_left_.empty()) {
    cv::remap(left_gray, left_rect, rect_map1_left_, rect_map2_left_, cv::INTER_LINEAR);
    cv::remap(right_gray, right_rect, rect_map1_right_, rect_map2_right_, cv::INTER_LINEAR);
    left_gray = left_rect;
    right_gray = right_rect;
  }

  // Build cuVSLAM images
  int64_t timestamp_ns = stamp_to_ns(left_msg->header.stamp);

  cuvslam::Image img0;
  img0.pixels = left_gray.data;
  img0.width = left_gray.cols;
  img0.height = left_gray.rows;
  img0.pitch = left_gray.step;
  img0.encoding = cuvslam::ImageData::Encoding::MONO;
  img0.data_type = cuvslam::ImageData::DataType::UINT8;
  img0.is_gpu_mem = false;
  img0.timestamp_ns = timestamp_ns;
  img0.camera_index = 0;

  cuvslam::Image img1;
  img1.pixels = right_gray.data;
  img1.width = right_gray.cols;
  img1.height = right_gray.rows;
  img1.pitch = right_gray.step;
  img1.encoding = cuvslam::ImageData::Encoding::MONO;
  img1.data_type = cuvslam::ImageData::DataType::UINT8;
  img1.is_gpu_mem = false;
  img1.timestamp_ns = timestamp_ns;
  img1.camera_index = 1;

  cuvslam::Odometry::ImageSet images = {img0, img1};

  // Register all IMU measurements up to this image's timestamp
  drain_imu_until(timestamp_ns);

  // Track
  cuvslam::PoseEstimate estimate;
  try {
    estimate = odometry_->Track(images);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Track failed: %s", e.what());
    return;
  }

  frame_count_++;

  if (!estimate.world_from_rig.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Tracking lost at frame %u", frame_count_);
    return;
  }

  // Optional SLAM tracking — replace VO pose with loop-closure-corrected pose
  if (slam_) {
    try {
      cuvslam::Odometry::State state;
      odometry_->GetState(state);
      cuvslam::Pose slam_pose = slam_->Track(state);
      estimate.world_from_rig.value().pose = slam_pose;
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "SLAM track failed: %s", e.what());
    }
    log_slam_status();
  }

  // Publish
  rclcpp::Time stamp = left_msg->header.stamp;
  publish_odometry(estimate, stamp);

  if (enable_landmarks_) {
    publish_landmarks(stamp);
  }

  if (frame_count_ % 100 == 0) {
    RCLCPP_INFO(get_logger(), "Frame %u tracked successfully", frame_count_);
  }
}

void CuvslamROS::mono_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
  if (!tracker_initialized_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Tracker not initialized. Waiting for camera info...");
    return;
  }

  std::lock_guard<std::mutex> lock(tracker_mutex_);

  cv::Mat gray;
  try {
    auto cv_img = cv_bridge::toCvShare(img_msg, "mono8");
    gray = cv_img->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
    return;
  }

  int64_t timestamp_ns = stamp_to_ns(img_msg->header.stamp);

  cuvslam::Image img0;
  img0.pixels = gray.data;
  img0.width = gray.cols;
  img0.height = gray.rows;
  img0.pitch = gray.step;
  img0.encoding = cuvslam::ImageData::Encoding::MONO;
  img0.data_type = cuvslam::ImageData::DataType::UINT8;
  img0.is_gpu_mem = false;
  img0.timestamp_ns = timestamp_ns;
  img0.camera_index = 0;

  cuvslam::Odometry::ImageSet images = {img0};

  cuvslam::PoseEstimate estimate;
  try {
    estimate = odometry_->Track(images);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Track failed: %s", e.what());
    return;
  }

  frame_count_++;

  if (!estimate.world_from_rig.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Tracking lost at frame %u", frame_count_);
    return;
  }

  if (slam_) {
    try {
      cuvslam::Odometry::State state;
      odometry_->GetState(state);
      cuvslam::Pose slam_pose = slam_->Track(state);
      estimate.world_from_rig.value().pose = slam_pose;
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "SLAM track failed: %s", e.what());
    }
    log_slam_status();
  }

  rclcpp::Time stamp = img_msg->header.stamp;
  publish_odometry(estimate, stamp);

  if (enable_landmarks_) {
    publish_landmarks(stamp);
  }

  if (frame_count_ % 100 == 0) {
    RCLCPP_INFO(get_logger(), "Frame %u tracked successfully", frame_count_);
  }
}

void CuvslamROS::rgbd_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {

  if (!tracker_initialized_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Tracker not initialized. Waiting for camera info...");
    return;
  }

  std::lock_guard<std::mutex> lock(tracker_mutex_);

  // Convert RGB to grayscale
  cv::Mat gray;
  try {
    auto rgb_cv = cv_bridge::toCvShare(rgb_msg, "mono8");
    gray = rgb_cv->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge error (rgb): %s", e.what());
    return;
  }

  // Depth image (16UC1)
  cv::Mat depth;
  try {
    auto depth_cv = cv_bridge::toCvShare(depth_msg);
    depth = depth_cv->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge error (depth): %s", e.what());
    return;
  }

  int64_t timestamp_ns = stamp_to_ns(rgb_msg->header.stamp);

  // RGB image
  cuvslam::Image img0;
  img0.pixels = gray.data;
  img0.width = gray.cols;
  img0.height = gray.rows;
  img0.pitch = gray.step;
  img0.encoding = cuvslam::ImageData::Encoding::MONO;
  img0.data_type = cuvslam::ImageData::DataType::UINT8;
  img0.is_gpu_mem = false;
  img0.timestamp_ns = timestamp_ns;
  img0.camera_index = 0;

  cuvslam::Odometry::ImageSet images = {img0};

  // Depth image
  cuvslam::Image depth_img;
  depth_img.pixels = depth.data;
  depth_img.width = depth.cols;
  depth_img.height = depth.rows;
  depth_img.pitch = depth.step;
  depth_img.encoding = cuvslam::ImageData::Encoding::MONO;
  depth_img.data_type = (depth.type() == CV_32FC1)
    ? cuvslam::ImageData::DataType::FLOAT32
    : cuvslam::ImageData::DataType::UINT16;
  depth_img.is_gpu_mem = false;
  depth_img.timestamp_ns = timestamp_ns;
  depth_img.camera_index = 0;

  cuvslam::Odometry::ImageSet depths = {depth_img};

  // Register all IMU measurements up to this image's timestamp
  drain_imu_until(timestamp_ns);

  // Track with depth
  cuvslam::PoseEstimate estimate;
  try {
    estimate = odometry_->Track(images, {}, depths);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Track failed: %s", e.what());
    return;
  }

  frame_count_++;

  if (!estimate.world_from_rig.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Tracking lost at frame %u", frame_count_);
    return;
  }

  rclcpp::Time stamp = rgb_msg->header.stamp;
  publish_odometry(estimate, stamp);

  if (enable_landmarks_) {
    publish_landmarks(stamp);
  }

  if (frame_count_ % 100 == 0) {
    RCLCPP_INFO(get_logger(), "Frame %u tracked successfully", frame_count_);
  }
}

void CuvslamROS::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  if (!tracker_initialized_ || !odometry_) return;

  cuvslam::ImuMeasurement imu;
  imu.timestamp_ns = stamp_to_ns(msg->header.stamp);
  imu.linear_accelerations = {
    static_cast<float>(msg->linear_acceleration.x),
    static_cast<float>(msg->linear_acceleration.y),
    static_cast<float>(msg->linear_acceleration.z)
  };
  imu.angular_velocities = {
    static_cast<float>(msg->angular_velocity.x),
    static_cast<float>(msg->angular_velocity.y),
    static_cast<float>(msg->angular_velocity.z)
  };

  // Buffer only; drained in image callback just before Track() so the shared
  // cuVSLAM timestamp cursor stays monotonic w.r.t. frames.
  std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
  if (imu.timestamp_ns <= last_registered_imu_ts_) return;  // drop stale IMU
  imu_buffer_.push_back(imu);
}

void CuvslamROS::drain_imu_until(int64_t image_ts_ns) {
  if (!odometry_) return;
  std::deque<cuvslam::ImuMeasurement> to_register;
  {
    std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
    while (!imu_buffer_.empty() && imu_buffer_.front().timestamp_ns <= image_ts_ns) {
      to_register.push_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
  }
  for (const auto& imu : to_register) {
    try {
      odometry_->RegisterImuMeasurement(0, imu);
      last_registered_imu_ts_ = imu.timestamp_ns;
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "IMU registration failed: %s", e.what());
    }
  }
}

void CuvslamROS::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
  if (camera_info_received_) return;

  image_width_ = msg->width;
  image_height_ = msg->height;
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_ = msg->k[2];
  cy_ = msg->k[5];

  // Parse distortion
  if (msg->distortion_model == "equidistant" || msg->distortion_model == "fisheye") {
    distortion_model_ = "fisheye";
  } else if (msg->distortion_model == "plumb_bob" || msg->distortion_model == "rational_polynomial") {
    distortion_model_ = "brown";
  }
  distortion_coeffs_.assign(msg->d.begin(), msg->d.end());

  // Use P matrix for baseline if available: P[3] = -fx * baseline
  if (msg->p[3] != 0.0) {
    rig_from_cam1_tx_ = std::abs(msg->p[3] / msg->p[0]);
  }

  RCLCPP_INFO(get_logger(), "Got CameraInfo: %dx%d, fx=%.1f, baseline=%.3fm",
    image_width_, image_height_, fx_, rig_from_cam1_tx_);

  camera_info_received_ = true;

  setup_camera_rig();
  setup_tracker();
  tracker_initialized_ = true;

  // Unsubscribe from camera info
  camera_info_sub_.reset();
}

// ============================================================================
// Publishing
// ============================================================================

void CuvslamROS::log_slam_status() {
  if (!slam_) return;
  try {
    cuvslam::Slam::Metrics metrics;
    slam_->GetSlamMetrics(metrics);
    std::vector<cuvslam::PoseStamped> lc_poses;
    slam_->GetLoopClosurePoses(lc_poses);

    // New loop closure event detected
    if (lc_poses.size() > last_lc_pose_count_) {
      const auto& last = lc_poses.back();
      RCLCPP_WARN(get_logger(),
        "*** LOOP CLOSURE #%zu fired @ t=%.3fs  (tracked=%u, pnp=%u, good=%u) ***",
        lc_poses.size(),
        last.timestamp_ns / 1e9,
        metrics.lc_tracked_landmarks_count,
        metrics.lc_pnp_landmarks_count,
        metrics.lc_good_landmarks_count);
      last_lc_pose_count_ = lc_poses.size();
    }
    // Periodic status
    if (frame_count_ % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "SLAM status: lc_events=%zu  lc_active=%d  pgo_active=%d  lc_selected=%u  lc_tracked=%u",
        lc_poses.size(), (int)metrics.lc_status, (int)metrics.pgo_status,
        metrics.lc_selected_landmarks_count, metrics.lc_tracked_landmarks_count);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "SLAM status query failed: %s", e.what());
  }
}

void CuvslamROS::publish_odometry(const cuvslam::PoseEstimate& estimate, const rclcpp::Time& stamp) {
  const auto& cv_pose = estimate.world_from_rig.value().pose;
  const auto& cov = estimate.world_from_rig.value().covariance;
  auto ros_pose = opencv_to_ros_pose(cv_pose);

  // nav_msgs/Odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id = base_frame_id_;
  odom_msg.pose.pose = ros_pose;

  // Copy covariance (cuVSLAM: rot_x, rot_y, rot_z, x, y, z -> ROS: x, y, z, rot_x, rot_y, rot_z)
  // cuVSLAM order: [rx, ry, rz, tx, ty, tz], ROS order: [tx, ty, tz, rx, ry, rz]
  // Reorder the 6x6 covariance matrix
  const int cv_to_ros[6] = {3, 4, 5, 0, 1, 2};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      odom_msg.pose.covariance[i * 6 + j] = cov[cv_to_ros[i] * 6 + cv_to_ros[j]];
    }
  }
  odom_pub_->publish(odom_msg);

  // PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = odom_frame_id_;
  pose_msg.pose = ros_pose;
  pose_pub_->publish(pose_msg);

  // Path
  path_msg_.header.stamp = stamp;
  if (path_msg_.poses.size() > 10000) {
    path_msg_.poses.erase(path_msg_.poses.begin());
  }
  path_msg_.poses.push_back(pose_msg);
  path_pub_->publish(path_msg_);

  // TF: odom -> base_link
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_id_;
  tf.child_frame_id = base_frame_id_;
  tf.transform.translation.x = ros_pose.position.x;
  tf.transform.translation.y = ros_pose.position.y;
  tf.transform.translation.z = ros_pose.position.z;
  tf.transform.rotation = ros_pose.orientation;
  tf_broadcaster_->sendTransform(tf);

  if (trajectory_file_.is_open()) {
    const double t_sec = stamp.seconds();
    trajectory_file_ << std::setprecision(9) << t_sec << " "
                     << std::setprecision(6)
                     << ros_pose.position.x << " "
                     << ros_pose.position.y << " "
                     << ros_pose.position.z << " "
                     << ros_pose.orientation.x << " "
                     << ros_pose.orientation.y << " "
                     << ros_pose.orientation.z << " "
                     << ros_pose.orientation.w << "\n";
  }
}

void CuvslamROS::publish_landmarks(const rclcpp::Time& stamp) {
  if (!landmark_pub_ || !odometry_) return;

  visualization_msgs::msg::MarkerArray markers;

  auto make_marker = [&](int id, const std::string& ns,
                         float r, float g, float b, float size) {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = odom_frame_id_;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = size;
    m.scale.y = size;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
    return m;
  };

  auto cv_to_ros = [](const cuvslam::Vector3f& c) {
    geometry_msgs::msg::Point p;
    p.x = c[2];   // cv_z -> ros_x
    p.y = -c[0];  // -cv_x -> ros_y
    p.z = -c[1];  // -cv_y -> ros_z
    return p;
  };

  // (1) SLAM map landmarks (persistent, usable in mono mode)
  if (slam_) {
    try {
      auto map_lm = slam_->ReadLandmarks(cuvslam::Slam::DataLayer::Map);
      if (map_lm && !map_lm->landmarks.empty()) {
        auto m = make_marker(0, "cuvslam_map_landmarks", 0.3f, 0.8f, 0.3f, 0.02f);
        for (const auto& lm : map_lm->landmarks) m.points.push_back(cv_to_ros(lm.coords));
        markers.markers.push_back(m);
      }
      auto cur_lm = slam_->ReadLandmarks(cuvslam::Slam::DataLayer::Landmarks);
      if (cur_lm && !cur_lm->landmarks.empty()) {
        auto m = make_marker(1, "cuvslam_current_landmarks", 1.0f, 0.9f, 0.1f, 0.05f);
        for (const auto& lm : cur_lm->landmarks) m.points.push_back(cv_to_ros(lm.coords));
        markers.markers.push_back(m);
      }
    } catch (...) {}
  }

  // (2) Fallback: odometry's last-frame landmarks (stereo/RGBD only — empty in mono)
  if (markers.markers.empty()) {
    try {
      auto odo_lm = odometry_->GetLastLandmarks();
      if (!odo_lm.empty()) {
        auto m = make_marker(2, "cuvslam_odo_landmarks", 0.0f, 1.0f, 0.0f, 0.02f);
        for (const auto& lm : odo_lm) m.points.push_back(cv_to_ros(lm.coords));
        markers.markers.push_back(m);
      }
    } catch (...) {}
  }

  if (!markers.markers.empty()) landmark_pub_->publish(markers);
}

// ============================================================================
// Utilities
// ============================================================================

geometry_msgs::msg::Pose CuvslamROS::opencv_to_ros_pose(const cuvslam::Pose& cv_pose) {
  // cuVSLAM OpenCV convention: x-right, y-down, z-forward
  // ROS convention: x-forward, y-left, z-up
  //
  // Rotation from OpenCV to ROS frame:
  // ros_x =  cv_z   (forward)
  // ros_y = -cv_x   (left)
  // ros_z = -cv_y   (up)

  // cuVSLAM quaternion: (x, y, z, w)
  Eigen::Quaternionf q_cv(cv_pose.rotation[3], cv_pose.rotation[0],
                           cv_pose.rotation[1], cv_pose.rotation[2]);
  Eigen::Vector3f t_cv(cv_pose.translation[0], cv_pose.translation[1], cv_pose.translation[2]);

  // T_cv2ros rotates from OpenCV frame to ROS frame
  // [ros_x]   [ 0  0  1] [cv_x]
  // [ros_y] = [-1  0  0] [cv_y]
  // [ros_z]   [ 0 -1  0] [cv_z]
  Eigen::Matrix3f R_cv2ros;
  R_cv2ros <<  0,  0,  1,
              -1,  0,  0,
               0, -1,  0;

  // Transform: T_ros = R_cv2ros * T_cv * R_cv2ros^T
  Eigen::Matrix3f R_ros = R_cv2ros * q_cv.toRotationMatrix() * R_cv2ros.transpose();
  Eigen::Vector3f t_ros = R_cv2ros * t_cv;

  Eigen::Quaternionf q_ros(R_ros);

  geometry_msgs::msg::Pose pose;
  pose.position.x = t_ros.x();
  pose.position.y = t_ros.y();
  pose.position.z = t_ros.z();
  pose.orientation.x = q_ros.x();
  pose.orientation.y = q_ros.y();
  pose.orientation.z = q_ros.z();
  pose.orientation.w = q_ros.w();
  return pose;
}

int64_t CuvslamROS::stamp_to_ns(const rclcpp::Time& stamp) {
  return stamp.nanoseconds();
}

}  // namespace cuvslam_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cuvslam_ros::CuvslamROS)
