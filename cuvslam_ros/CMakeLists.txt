cmake_minimum_required(VERSION 3.26)
project(cuvslam_ros VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- cuVSLAM ----
# Point this to the cuVSLAM build directory
# Set via: -DCUVSLAM_BUILD_DIR=<path> or environment variable
if(NOT CUVSLAM_BUILD_DIR)
  if(DEFINED ENV{CUVSLAM_BUILD_DIR})
    set(CUVSLAM_BUILD_DIR $ENV{CUVSLAM_BUILD_DIR})
  else()
    set(CUVSLAM_BUILD_DIR "$ENV{HOME}/Documents/GitHub/slam/cuVSLAM/build"
        CACHE PATH "Path to cuVSLAM build directory")
  endif()
endif()

set(CUVSLAM_SRC_DIR "$ENV{HOME}/Documents/GitHub/slam/cuVSLAM"
    CACHE PATH "Path to cuVSLAM source directory")

if(NOT EXISTS "${CUVSLAM_SRC_DIR}/libs/cuvslam/cuvslam2.h")
  message(FATAL_ERROR "cuVSLAM source not found at ${CUVSLAM_SRC_DIR}")
endif()

message(STATUS "cuVSLAM source dir: ${CUVSLAM_SRC_DIR}")
message(STATUS "cuVSLAM build dir: ${CUVSLAM_BUILD_DIR}")

# cuVSLAM include path
set(CUVSLAM_INCLUDE_DIR "${CUVSLAM_SRC_DIR}/libs/cuvslam")

# cuVSLAM library - link against the shared lib (libcuvslam.so)
find_library(CUVSLAM_LIBRARY
  NAMES cuvslam
  PATHS "${CUVSLAM_BUILD_DIR}/bin"
        "${CUVSLAM_BUILD_DIR}/libs/cuvslam"
        "${CUVSLAM_BUILD_DIR}/lib"
        "${CUVSLAM_BUILD_DIR}"
  NO_DEFAULT_PATH
)

if(NOT CUVSLAM_LIBRARY)
  message(WARNING "libcuvslam not found in ${CUVSLAM_BUILD_DIR}. "
    "Build cuVSLAM first, then set CUVSLAM_BUILD_DIR.")
endif()

# ---- cuvslam_ros library ----
add_library(cuvslam_ros SHARED
  src/cuvslam_ros/cuvslam_ros.cpp
)
target_include_directories(cuvslam_ros PUBLIC
  include
  ${CUVSLAM_INCLUDE_DIR}
  ${CUVSLAM_SRC_DIR}/libs
)
target_link_libraries(cuvslam_ros
  ${CUVSLAM_LIBRARY}
  Eigen3::Eigen
)
ament_target_dependencies(cuvslam_ros
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  image_transport
  message_filters
  visualization_msgs
  OpenCV
)
rclcpp_components_register_nodes(cuvslam_ros "cuvslam_ros::CuvslamROS")

# ---- cuvslam_rosnode executable ----
add_executable(cuvslam_rosnode src/cuvslam_rosnode.cpp)
target_link_libraries(cuvslam_rosnode cuvslam_ros)
ament_target_dependencies(cuvslam_rosnode rclcpp)

# ---- Install ----
install(TARGETS cuvslam_ros cuvslam_rosnode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
