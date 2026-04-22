#ifndef __MONOCULAR_INERTIAL_NODE_HPP__
#define __MONOCULAR_INERTIAL_NODE_HPP__

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

class MonocularInertialNode : public rclcpp::Node
{
public:
    MonocularInertialNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoEqual, const string &strUseCompressed);
    ~MonocularInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void GrabCompressedImage(const CompressedImageMsg::SharedPtr msg);
    cv::Mat GetCompressedImage(const CompressedImageMsg::SharedPtr msg);

    void SyncWithImu();
    void PublishSLAMOutputs(const Sophus::SE3f &Tcw, double tIm);

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRaw_;
    rclcpp::Subscription<CompressedImageMsg>::SharedPtr subImgCompressed_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapPoints_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    nav_msgs::msg::Path pathMsg_;
    std::string worldFrame_ = "map";
    std::string cameraFrame_ = "orb_camera";

    ORB_SLAM3::System *SLAM_;
    std::thread *syncThread_;

    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    queue<ImageMsg::SharedPtr> imgBufRaw_;
    queue<CompressedImageMsg::SharedPtr> imgBufCompressed_;
    std::mutex bufMutexImg_;

    bool doEqual_;
    bool useCompressed_;
    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_;

    // Auto-detect IMU accelerometer unit (g vs m/s^2).
    // Uses mean ||acc|| of the first N samples: ~9.81 -> m/s^2 (scale 1),
    // ~1.0 -> g (scale 9.80665). Everything else defaults to 1 with a warning.
    bool accelScaleResolved_ = false;
    double accelScale_ = 1.0;
    int accelDetectCount_ = 0;
    double accelMagSum_ = 0.0;
    std::vector<ImuMsg::SharedPtr> accelDetectBuf_;
    static constexpr int kAccelDetectSamples = 40;
};

#endif
