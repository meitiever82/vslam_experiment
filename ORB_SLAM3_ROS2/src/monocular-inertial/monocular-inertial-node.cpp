#include "monocular-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "MapPoint.h"

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoEqual, const string &strUseCompressed) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    stringstream ss_comp(strUseCompressed);
    ss_comp >> boolalpha >> useCompressed_;

    bClahe_ = doEqual_;
    if (bClahe_) {
        clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
    }
    std::cout << "Equal: " << doEqual_ << std::endl;
    std::cout << "Use Compressed: " << useCompressed_ << std::endl;

    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));

    if (useCompressed_)
    {
        subImgCompressed_ = this->create_subscription<CompressedImageMsg>("camera", 100, std::bind(&MonocularInertialNode::GrabCompressedImage, this, _1));
    }
    else
    {
        subImgRaw_ = this->create_subscription<ImageMsg>("camera", 100, std::bind(&MonocularInertialNode::GrabImage, this, _1));
    }

    pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    pubPath_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    pubMapPoints_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/map_points", 10);
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pathMsg_.header.frame_id = worldFrame_;

    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);
}

void MonocularInertialNode::PublishSLAMOutputs(const Sophus::SE3f &Tcw, double tIm)
{
    if (Tcw.matrix().isZero(0))
        return;

    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q(Twc.unit_quaternion());

    rclcpp::Time stamp(static_cast<int64_t>(tIm * 1e9));

    geometry_msgs::msg::TransformStamped tfMsg;
    tfMsg.header.stamp = stamp;
    tfMsg.header.frame_id = worldFrame_;
    tfMsg.child_frame_id = cameraFrame_;
    tfMsg.transform.translation.x = t.x();
    tfMsg.transform.translation.y = t.y();
    tfMsg.transform.translation.z = t.z();
    tfMsg.transform.rotation.x = q.x();
    tfMsg.transform.rotation.y = q.y();
    tfMsg.transform.rotation.z = q.z();
    tfMsg.transform.rotation.w = q.w();
    tfBroadcaster_->sendTransform(tfMsg);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = worldFrame_;
    odom.child_frame_id = cameraFrame_;
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation = tfMsg.transform.rotation;
    pubOdom_->publish(odom);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose = odom.pose.pose;
    pathMsg_.header.stamp = stamp;
    pathMsg_.poses.push_back(ps);
    pubPath_->publish(pathMsg_);

    auto mps = SLAM_->GetTrackedMapPoints();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = worldFrame_;
    cloud.height = 1;
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(mps.size());
    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    size_t valid = 0;
    for (auto *mp : mps) {
        if (!mp || mp->isBad()) continue;
        Eigen::Vector3f p = mp->GetWorldPos();
        *ix = p.x(); *iy = p.y(); *iz = p.z();
        ++ix; ++iy; ++iz; ++valid;
    }
    mod.resize(valid);
    pubMapPoints_->publish(cloud);
}

MonocularInertialNode::~MonocularInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    try { SLAM_->SaveTrajectoryTUM("FrameTrajectory.txt"); }
    catch (const std::exception &e) { std::cerr << "SaveTrajectoryTUM failed: " << e.what() << std::endl; }
    catch (...) { std::cerr << "SaveTrajectoryTUM failed (unknown)" << std::endl; }
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    if (!accelScaleResolved_)
    {
        const auto &a = msg->linear_acceleration;
        accelMagSum_ += std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        accelDetectBuf_.push_back(msg);
        ++accelDetectCount_;

        if (accelDetectCount_ < kAccelDetectSamples)
            return;

        double mean = accelMagSum_ / accelDetectCount_;
        if (mean > 7.0 && mean < 12.0)
            accelScale_ = 1.0;
        else if (mean > 0.7 && mean < 1.2)
            accelScale_ = 9.80665;
        else {
            RCLCPP_WARN(this->get_logger(),
                        "Accel magnitude %.3f outside expected bands; leaving scale=1.0", mean);
            accelScale_ = 1.0;
        }
        RCLCPP_INFO(this->get_logger(),
                    "IMU accel unit detected (mean ||a||=%.3f over %d samples) -> scale=%.5f",
                    mean, accelDetectCount_, accelScale_);
        accelScaleResolved_ = true;

        bufMutex_.lock();
        for (auto &m : accelDetectBuf_) {
            m->linear_acceleration.x *= accelScale_;
            m->linear_acceleration.y *= accelScale_;
            m->linear_acceleration.z *= accelScale_;
            imuBuf_.push(m);
        }
        bufMutex_.unlock();
        accelDetectBuf_.clear();
        return;
    }

    msg->linear_acceleration.x *= accelScale_;
    msg->linear_acceleration.y *= accelScale_;
    msg->linear_acceleration.z *= accelScale_;

    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexImg_.lock();

    if (!imgBufRaw_.empty())
        imgBufRaw_.pop();
    imgBufRaw_.push(msg);

    bufMutexImg_.unlock();
}

cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialNode::GrabCompressedImage(const CompressedImageMsg::SharedPtr msg)
{
    bufMutexImg_.lock();

    if (!imgBufCompressed_.empty())
        imgBufCompressed_.pop();
    imgBufCompressed_.push(msg);

    bufMutexImg_.unlock();
}

cv::Mat MonocularInertialNode::GetCompressedImage(const CompressedImageMsg::SharedPtr msg)
{
    // Decode compressed image to cv::Mat
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);

    if (image.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
    }

    return image;
}

void MonocularInertialNode::SyncWithImu()
{
    static double lastImgTime = 0;

    while (1)
    {
        cv::Mat im;
        double tIm = 0;

        bool hasImage = false;
        if (useCompressed_)
        {
            hasImage = !imgBufCompressed_.empty() && !imuBuf_.empty();
        }
        else
        {
            hasImage = !imgBufRaw_.empty() && !imuBuf_.empty();
        }

        if (hasImage)
        {
            if (useCompressed_)
            {
                tIm = Utility::StampToSec(imgBufCompressed_.front()->header.stamp);
            }
            else
            {
                tIm = Utility::StampToSec(imgBufRaw_.front()->header.stamp);
            }

            // Detect timestamp jump (e.g., rosbag loop) and clear IMU buffer
            if (lastImgTime > 0 && tIm < lastImgTime - 1.0)
            {
                std::cout << "Timestamp jump detected (rosbag loop?): clearing IMU buffer. lastImgTime="
                          << std::fixed << lastImgTime << ", tIm=" << tIm << std::endl;
                bufMutex_.lock();
                while (!imuBuf_.empty())
                    imuBuf_.pop();
                bufMutex_.unlock();
                lastImgTime = tIm;
                continue;
            }
            lastImgTime = tIm;

            double tImuBack = Utility::StampToSec(imuBuf_.back()->header.stamp);
            // Allow small time offset (1 IMU period = 10ms at 100Hz) between image and IMU
            const double timeOffset = 0.01;
            if (tIm > tImuBack + timeOffset)
            {
                continue;
            }

            bufMutexImg_.lock();
            if (useCompressed_)
            {
                im = GetCompressedImage(imgBufCompressed_.front());
                imgBufCompressed_.pop();
            }
            else
            {
                im = GetImage(imgBufRaw_.front());
                imgBufRaw_.pop();
            }
            bufMutexImg_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(im, im);
            }

            Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, tIm, vImuMeas);
            PublishSLAMOutputs(Tcw, tIm);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
