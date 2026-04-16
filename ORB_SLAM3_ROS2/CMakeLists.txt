cmake_minimum_required(VERSION 3.5)
project(orbslam3_ros2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)

# Use system OpenCV (matches rebuilt ORB_SLAM3)
find_package(OpenCV 4.5 REQUIRED)

find_package(cv_bridge REQUIRED)

# Use Sophus from ORB-SLAM3's Thirdparty (version 1.1.0)
set(Sophus_DIR "/home/steve/Documents/GitHub/slam/ORB_SLAM3/Thirdparty/Sophus/build")
find_package(Sophus REQUIRED)

find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

# Force RPATH to use our rebuilt ORB_SLAM3 (system OpenCV), not the one in /opt/fusion
set(CMAKE_INSTALL_RPATH "${ORB_SLAM3_ROOT_DIR}/lib;${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs ORB_SLAM3 Pangolin)
ament_target_dependencies(mono cv_bridge)
target_link_libraries(mono ${OpenCV_LIBS})

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs message_filters ORB_SLAM3 Pangolin)
ament_target_dependencies(rgbd cv_bridge)
target_link_libraries(rgbd ${OpenCV_LIBS})

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs message_filters ORB_SLAM3 Pangolin)
ament_target_dependencies(stereo cv_bridge)
target_link_libraries(stereo ${OpenCV_LIBS})

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros ORB_SLAM3 Pangolin)
ament_target_dependencies(stereo-inertial cv_bridge)
target_link_libraries(stereo-inertial ${OpenCV_LIBS})

add_executable(mono-inertial
  src/monocular-inertial/monocular-inertial.cpp
  src/monocular-inertial/monocular-inertial-node.cpp
)
ament_target_dependencies(mono-inertial rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros ORB_SLAM3 Pangolin)
ament_target_dependencies(mono-inertial cv_bridge)
target_link_libraries(mono-inertial ${OpenCV_LIBS})

install(TARGETS mono rgbd stereo stereo-inertial mono-inertial
  DESTINATION lib/${PROJECT_NAME})

#Install launch files.
install(DIRECTORY launch config vocabulary rviz
 DESTINATION share/${PROJECT_NAME}/)

ament_package()

