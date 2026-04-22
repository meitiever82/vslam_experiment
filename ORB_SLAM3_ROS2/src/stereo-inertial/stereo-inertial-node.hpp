#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual, const string &strUseCompressed);
    ~StereoInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    // Raw image callbacks
    void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
    void GrabImageRight(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    // Compressed image callbacks
    void GrabCompressedImageLeft(const CompressedImageMsg::SharedPtr msgLeft);
    void GrabCompressedImageRight(const CompressedImageMsg::SharedPtr msgRight);
    cv::Mat GetCompressedImage(const CompressedImageMsg::SharedPtr msg);

    void SyncWithImu();
    void PublishSLAMOutputs(const Sophus::SE3f &Tcw, double tIm);

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    // Raw image subscriptions
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeftRaw_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRightRaw_;
    // Compressed image subscriptions
    rclcpp::Subscription<CompressedImageMsg>::SharedPtr subImgLeftCompressed_;
    rclcpp::Subscription<CompressedImageMsg>::SharedPtr subImgRightCompressed_;

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image buffers (use variant to store either type)
    queue<ImageMsg::SharedPtr> imgLeftBufRaw_, imgRightBufRaw_;
    queue<CompressedImageMsg::SharedPtr> imgLeftBufCompressed_, imgRightBufCompressed_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    bool doRectify_;
    bool doEqual_;
    bool useCompressed_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_;

    // ROS publishers for SLAM outputs (pose, trajectory, map points).
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapPoints_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    nav_msgs::msg::Path pathMsg_;
    std::string worldFrame_ = "map";
    std::string cameraFrame_ = "orb_camera";

    // Auto-detect IMU accelerometer unit (g vs m/s^2); mirrors mono-inertial.
    bool accelScaleResolved_ = false;
    double accelScale_ = 1.0;
    int accelDetectCount_ = 0;
    double accelMagSum_ = 0.0;
    std::vector<ImuMsg::SharedPtr> accelDetectBuf_;
    static constexpr int kAccelDetectSamples = 40;
};

#endif
