#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "MapPoint.h"

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual, const string &strUseCompressed) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    stringstream ss_comp(strUseCompressed);
    ss_comp >> boolalpha >> useCompressed_;

    bClahe_ = doEqual_;
    if (bClahe_) {
        clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
    }
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;
    std::cout << "Use Compressed: " << useCompressed_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));

    if (useCompressed_)
    {
        subImgLeftCompressed_ = this->create_subscription<CompressedImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabCompressedImageLeft, this, _1));
        subImgRightCompressed_ = this->create_subscription<CompressedImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabCompressedImageRight, this, _1));
    }
    else
    {
        subImgLeftRaw_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
        subImgRightRaw_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));
    }

    pubOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
    pubPath_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    pubMapPoints_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/map_points", 10);
    tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pathMsg_.header.frame_id = worldFrame_;

    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

void StereoInertialNode::PublishSLAMOutputs(const Sophus::SE3f &Tcw, double tIm)
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

StereoInertialNode::~StereoInertialNode()
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

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
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

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBufRaw_.empty())
        imgLeftBufRaw_.pop();
    imgLeftBufRaw_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBufRaw_.empty())
        imgRightBufRaw_.pop();
    imgRightBufRaw_.push(msgRight);

    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
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

void StereoInertialNode::GrabCompressedImageLeft(const CompressedImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBufCompressed_.empty())
        imgLeftBufCompressed_.pop();
    imgLeftBufCompressed_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabCompressedImageRight(const CompressedImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBufCompressed_.empty())
        imgRightBufCompressed_.pop();
    imgRightBufCompressed_.push(msgRight);

    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetCompressedImage(const CompressedImageMsg::SharedPtr msg)
{
    // Decode compressed image to cv::Mat
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);

    if (image.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
    }

    return image;
}

void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;

        bool hasImages = false;
        if (useCompressed_)
        {
            hasImages = !imgLeftBufCompressed_.empty() && !imgRightBufCompressed_.empty() && !imuBuf_.empty();
        }
        else
        {
            hasImages = !imgLeftBufRaw_.empty() && !imgRightBufRaw_.empty() && !imuBuf_.empty();
        }

        if (hasImages)
        {
            if (useCompressed_)
            {
                tImLeft = Utility::StampToSec(imgLeftBufCompressed_.front()->header.stamp);
                tImRight = Utility::StampToSec(imgRightBufCompressed_.front()->header.stamp);

                bufMutexRight_.lock();
                while ((tImLeft - tImRight) > maxTimeDiff && imgRightBufCompressed_.size() > 1)
                {
                    imgRightBufCompressed_.pop();
                    tImRight = Utility::StampToSec(imgRightBufCompressed_.front()->header.stamp);
                }
                bufMutexRight_.unlock();

                bufMutexLeft_.lock();
                while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBufCompressed_.size() > 1)
                {
                    imgLeftBufCompressed_.pop();
                    tImLeft = Utility::StampToSec(imgLeftBufCompressed_.front()->header.stamp);
                }
                bufMutexLeft_.unlock();
            }
            else
            {
                tImLeft = Utility::StampToSec(imgLeftBufRaw_.front()->header.stamp);
                tImRight = Utility::StampToSec(imgRightBufRaw_.front()->header.stamp);

                bufMutexRight_.lock();
                while ((tImLeft - tImRight) > maxTimeDiff && imgRightBufRaw_.size() > 1)
                {
                    imgRightBufRaw_.pop();
                    tImRight = Utility::StampToSec(imgRightBufRaw_.front()->header.stamp);
                }
                bufMutexRight_.unlock();

                bufMutexLeft_.lock();
                while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBufRaw_.size() > 1)
                {
                    imgLeftBufRaw_.pop();
                    tImLeft = Utility::StampToSec(imgLeftBufRaw_.front()->header.stamp);
                }
                bufMutexLeft_.unlock();
            }

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            if (useCompressed_)
            {
                bufMutexLeft_.lock();
                imLeft = GetCompressedImage(imgLeftBufCompressed_.front());
                imgLeftBufCompressed_.pop();
                bufMutexLeft_.unlock();

                bufMutexRight_.lock();
                imRight = GetCompressedImage(imgRightBufCompressed_.front());
                imgRightBufCompressed_.pop();
                bufMutexRight_.unlock();
            }
            else
            {
                bufMutexLeft_.lock();
                imLeft = GetImage(imgLeftBufRaw_.front());
                imgLeftBufRaw_.pop();
                bufMutexLeft_.unlock();

                bufMutexRight_.lock();
                imRight = GetImage(imgRightBufRaw_.front());
                imgRightBufRaw_.pop();
                bufMutexRight_.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
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
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            Sophus::SE3f Tcw = SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
            PublishSLAMOutputs(Tcw, tImLeft);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
