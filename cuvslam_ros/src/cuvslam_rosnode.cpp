// Copyright (c) 2026, Steve. All rights reserved.
// cuVSLAM ROS2 node entry point

#include <rclcpp/rclcpp.hpp>
#include "cuvslam_ros/cuvslam_ros.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<cuvslam_ros::CuvslamROS>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
