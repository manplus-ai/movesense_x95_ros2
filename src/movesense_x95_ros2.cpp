// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros2/camera_config.h"
#include "movesense_x95_ros2/camera_driver.h"
#include "movesense_x95_ros2/imu_publisher.h"
#include "movesense_x95_ros2/imu_receiver.h"
#include "movesense_x95_ros2/stereo_calib.h"
#include "movesense_x95_ros2/stereo_publisher.h"
#include "movesense_x95_ros2/stereo_receiver.h"
#include "movesense_x95_ros2/tf_publisher.h"
#include "movesense_x95_ros2/thread_safe_queue.h"

#include <array>
#include <filesystem>
#include <chrono>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <thread>

using namespace movesense_x95_ros2;

static rclcpp::Logger Logger()
{
    return rclcpp::get_logger("movesense_x95_ros2");
}

static CameraConfig LoadConfig(rclcpp::Node::SharedPtr node)
{
    CameraConfig cfg;

    cfg.cameraIp = node->declare_parameter<std::string>("camera_ip", cfg.cameraIp);
    cfg.alignTimeOnStart = node->declare_parameter<bool>("align_time_on_start", cfg.alignTimeOnStart);
    cfg.firstFrameWaitMs = node->declare_parameter<int>("first_frame_wait_ms", cfg.firstFrameWaitMs);

    cfg.fps = node->declare_parameter<int>("depth_fps", cfg.fps);
    cfg.stereoWidth = node->declare_parameter<int>("stereo_width", cfg.stereoWidth);
    cfg.stereoHeight = node->declare_parameter<int>("stereo_height", cfg.stereoHeight);
    cfg.downsampleMode = node->declare_parameter<int>("downsample_mode", cfg.downsampleMode);
    cfg.depthWidth = node->declare_parameter<int>("depth_width", cfg.depthWidth);
    cfg.depthHeight = node->declare_parameter<int>("depth_height", cfg.depthHeight);
    cfg.colorWidth = node->declare_parameter<int>("color_width", cfg.colorWidth);
    cfg.colorHeight = node->declare_parameter<int>("color_height", cfg.colorHeight);

    cfg.enableStereo = node->declare_parameter<bool>("enable_stereo", cfg.enableStereo);
    cfg.enableDepth = node->declare_parameter<bool>("enable_depth", cfg.enableDepth);
    cfg.enableColor = node->declare_parameter<bool>("enable_color", cfg.enableColor);
    cfg.enableImu = node->declare_parameter<bool>("enable_imu", cfg.enableImu);
    cfg.enableDetection = node->declare_parameter<bool>("enable_detection", cfg.enableDetection);

    cfg.stereoAutoExpo = node->declare_parameter<bool>("stereo_auto_expo", cfg.stereoAutoExpo);
    cfg.stereoExposureUs = node->declare_parameter<int>("stereo_exposure_us", cfg.stereoExposureUs);
    cfg.stereoGainX = node->declare_parameter<int>("stereo_gain_x", cfg.stereoGainX);
    cfg.stereoMaxExposureUs = node->declare_parameter<int>("stereo_max_exposure_us", cfg.stereoMaxExposureUs);
    cfg.stereoMinExposureUs = node->declare_parameter<int>("stereo_min_exposure_us", cfg.stereoMinExposureUs);
    cfg.stereoMaxGainX = node->declare_parameter<int>("stereo_max_gain_x", cfg.stereoMaxGainX);
    cfg.stereoMinGainX = node->declare_parameter<int>("stereo_min_gain_x", cfg.stereoMinGainX);

    cfg.colorAutoExpo = node->declare_parameter<bool>("color_auto_expo", cfg.colorAutoExpo);
    cfg.colorExposureUs = node->declare_parameter<int>("color_exposure_us", cfg.colorExposureUs);
    cfg.colorGainX = node->declare_parameter<int>("color_gain_x", cfg.colorGainX);
    cfg.colorMaxExposureUs = node->declare_parameter<int>("color_max_exposure_us", cfg.colorMaxExposureUs);
    cfg.colorMinExposureUs = node->declare_parameter<int>("color_min_exposure_us", cfg.colorMinExposureUs);
    cfg.colorMaxGainX = node->declare_parameter<int>("color_max_gain_x", cfg.colorMaxGainX);
    cfg.colorMinGainX = node->declare_parameter<int>("color_min_gain_x", cfg.colorMinGainX);

    cfg.doePower = node->declare_parameter<int>("doe_power", cfg.doePower);
    cfg.registration = node->declare_parameter<int>("registration", cfg.registration);

    cfg.imuCalibDir = node->declare_parameter<std::string>("imu_calib_dir", cfg.imuCalibDir);

    cfg.cam0Topic = node->declare_parameter<std::string>("cam0_topic", cfg.cam0Topic);
    cfg.cam1Topic = node->declare_parameter<std::string>("cam1_topic", cfg.cam1Topic);
    cfg.colorTopic = node->declare_parameter<std::string>("color_topic", cfg.colorTopic);
    cfg.colorInfoTopic = node->declare_parameter<std::string>("color_info_topic", cfg.colorInfoTopic);
    cfg.depthTopic = node->declare_parameter<std::string>("depth_topic", cfg.depthTopic);
    cfg.depthInfoTopic = node->declare_parameter<std::string>("depth_info_topic", cfg.depthInfoTopic);
    cfg.detTopic = node->declare_parameter<std::string>("det_topic", cfg.detTopic);
    cfg.imuTopic = node->declare_parameter<std::string>("imu_topic", cfg.imuTopic);
    cfg.publishTf = node->declare_parameter<bool>("publish_tf", cfg.publishTf);
    cfg.baseFrameId = node->declare_parameter<std::string>("base_frame_id", cfg.baseFrameId);
    cfg.cam0FrameId = node->declare_parameter<std::string>("cam0_frame_id", cfg.cam0FrameId);
    cfg.cam1FrameId = node->declare_parameter<std::string>("cam1_frame_id", cfg.cam1FrameId);
    cfg.colorFrameId = node->declare_parameter<std::string>("color_frame_id", cfg.colorFrameId);
    cfg.detFrameId = node->declare_parameter<std::string>("det_frame_id", cfg.detFrameId);
    cfg.imuFrameId = node->declare_parameter<std::string>("imu_frame_id", cfg.imuFrameId);

    return cfg;
}

static sensor_msgs::msg::CameraInfo MakeCameraInfo(
    double fx1280, double fy1280, double cx1280, double cy1280, int width, int height, const std::string& frameId)
{
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frameId;
    info.width = static_cast<uint32_t>(width);
    info.height = static_cast<uint32_t>(height);
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);

    const double s = static_cast<double>(width) / 1280.0;
    const double fx = fx1280 * s;
    const double fy = fy1280 * s;
    const double cx = cx1280 * s;
    const double cy = cy1280 * s;

    info.k[0] = fx;
    info.k[1] = 0.0;
    info.k[2] = cx;
    info.k[3] = 0.0;
    info.k[4] = fy;
    info.k[5] = cy;
    info.k[6] = 0.0;
    info.k[7] = 0.0;
    info.k[8] = 1.0;

    info.r[0] = 1.0;
    info.r[1] = 0.0;
    info.r[2] = 0.0;
    info.r[3] = 0.0;
    info.r[4] = 1.0;
    info.r[5] = 0.0;
    info.r[6] = 0.0;
    info.r[7] = 0.0;
    info.r[8] = 1.0;

    info.p[0] = fx;
    info.p[1] = 0.0;
    info.p[2] = cx;
    info.p[3] = 0.0;
    info.p[4] = 0.0;
    info.p[5] = fy;
    info.p[6] = cy;
    info.p[7] = 0.0;
    info.p[8] = 0.0;
    info.p[9] = 0.0;
    info.p[10] = 1.0;
    info.p[11] = 0.0;

    return info;
}

static bool WriteMat(const std::string& path, const char* name, const cv::Mat& m)
{
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }
    fs << name << m;
    fs.release();
    return true;
}

static cv::Mat Intrinsics3x3(const std::array<double, 4>& k)
{
    cv::Mat m = cv::Mat::zeros(3, 3, CV_64F);
    m.at<double>(0, 0) = k[0];
    m.at<double>(1, 1) = k[1];
    m.at<double>(0, 2) = k[2];
    m.at<double>(1, 2) = k[3];
    m.at<double>(2, 2) = 1.0;
    return m;
}

static cv::Mat Distortion8x1(const std::array<double, 8>& d)
{
    cv::Mat m = cv::Mat::zeros(8, 1, CV_64F);
    for (int i = 0; i < 8; ++i) {
        m.at<double>(i, 0) = d[i];
    }
    return m;
}

static cv::Mat Matrix3x3(const std::array<double, 9>& r)
{
    cv::Mat m(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        m.at<double>(i / 3, i % 3) = r[i];
    }
    return m;
}

static cv::Mat Column3x1(const std::array<double, 3>& t)
{
    cv::Mat m(3, 1, CV_64F);
    for (int i = 0; i < 3; ++i) {
        m.at<double>(i, 0) = t[i];
    }
    return m;
}

static bool WriteStereoGeometryXml(const StereoCalib& calib, const std::string& dir)
{
    cv::Mat p = cv::Mat::zeros(3, 4, CV_64F);
    p.at<double>(0, 0) = calib.Fx();
    p.at<double>(1, 1) = calib.Fy();
    p.at<double>(0, 2) = calib.Cx();
    p.at<double>(1, 2) = calib.Cy();
    p.at<double>(0, 3) = calib.NegBaselineMm() * calib.Fx();
    p.at<double>(2, 2) = 1.0;

    bool ok = true;
    ok = WriteMat(dir + "/M1.xml", "M1", Intrinsics3x3(calib.LeftIntrinsics())) && ok;
    ok = WriteMat(dir + "/D1.xml", "D1", Distortion8x1(calib.LeftDistortion())) && ok;
    ok = WriteMat(dir + "/M2.xml", "M2", Intrinsics3x3(calib.RightIntrinsics())) && ok;
    ok = WriteMat(dir + "/D2.xml", "D2", Distortion8x1(calib.RightDistortion())) && ok;
    ok = WriteMat(dir + "/IR_1.xml", "IR_1", Matrix3x3(calib.LeftRectRotation())) && ok;
    ok = WriteMat(dir + "/IR_2.xml", "IR_2", Matrix3x3(calib.RightRectRotation())) && ok;
    ok = WriteMat(dir + "/t_P2.xml", "t_P2", p) && ok;

    return ok;
}

static bool WriteColorGeometryXml(const StereoCalib& calib, const std::string& dir)
{
    cv::Mat p = cv::Mat::zeros(3, 3, CV_64F);
    p.at<double>(0, 0) = calib.ColorFx();
    p.at<double>(1, 1) = calib.ColorFy();
    p.at<double>(0, 2) = calib.ColorCx();
    p.at<double>(1, 2) = calib.ColorCy();
    p.at<double>(2, 2) = 1.0;

    bool ok = true;
    ok = WriteMat(dir + "/M2.xml", "M2", Intrinsics3x3(calib.ColorIntrinsics())) && ok;
    ok = WriteMat(dir + "/D2.xml", "D2", Distortion8x1(calib.ColorDistortion())) && ok;
    ok = WriteMat(dir + "/t_P2.xml", "t_P2", p) && ok;
    ok = WriteMat(dir + "/iR_2.xml", "iR_2", Matrix3x3(calib.ColorInverseRectification())) && ok;
    ok = WriteMat(dir + "/T_reg.xml", "T_reg", Column3x1(calib.ColorTregMm())) && ok;
    ok = WriteMat(dir + "/R.xml", "R", Matrix3x3(calib.ColorRotation())) && ok;
    ok = WriteMat(dir + "/T.xml", "T", Column3x1(calib.ColorTranslationMm())) && ok;

    return ok;
}

static bool WriteOneImuCalibXml(const std::string& dir, const std::array<double, 9>& r, const std::array<double, 3>& t, double tsMs,
    double gyroNd, double gyroRw, double accNd, double accRw)
{
    std::filesystem::create_directories(dir);

    bool ok = true;
    cv::FileStorage fs;

    fs.open(dir + "/noise_IMU.xml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "gyroscope_noise_density" << gyroNd;
        fs << "gyroscope_random_walk" << gyroRw;
        fs << "accelerometer_noise_density" << accNd;
        fs << "accelerometer_random_walk" << accRw;
        fs.release();
    } else {
        ok = false;
    }

    fs.open(dir + "/ts_IMU.xml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "timeshift_cam_imu" << tsMs;
        fs.release();
    } else {
        ok = false;
    }

    cv::Mat R(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) {
        R.at<double>(i / 3, i % 3) = r[i];
    }
    fs.open(dir + "/r_IMU.xml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "r_IMU" << R;
        fs.release();
    } else {
        ok = false;
    }

    cv::Mat tv(1, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
        tv.at<double>(0, i) = t[i];
    }
    fs.open(dir + "/t_IMU.xml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "t_IMU" << tv;
        fs.release();
    } else {
        ok = false;
    }

    return ok;
}

static const char* CameraTypeName(int cameraType)
{
    if (cameraType == 1) {
        return "P";
    }
    if (cameraType == 2) {
        return "AP";
    }
    return "A";
}

static void WriteCalibXml(const StereoCalib& calib, const std::string& dir, int cameraType)
{
    if (dir.empty()) {
        return;
    }

    const char* typeName = CameraTypeName(cameraType);
    const bool imuOnColorOnly = (cameraType == 0);
    const bool hasColorLens = (cameraType != 1);

    if (!calib.HasStereoBlob()) {
        RCLCPP_WARN(Logger(), "[NODE] calib export: stereo calibration could not be read from a %s camera; nothing written", typeName);
        return;
    }

    if (!calib.IsValid()) {
        RCLCPP_WARN(Logger(), "[NODE] calib export: rectified intrinsics are invalid on a %s camera; the exported files reflect that", typeName);
    }

    try {
        const std::string sub = dir + "/xml";
        std::filesystem::create_directories(sub);

        if (WriteStereoGeometryXml(calib, sub)) {
            RCLCPP_INFO(Logger(), "[NODE] stereo calib xml written to %s (M1 D1 M2 D2 IR_1 IR_2 t_P2)", sub.c_str());
        } else {
            RCLCPP_ERROR(Logger(), "[NODE] stereo calib xml partially failed, dir=%s", sub.c_str());
        }

        if (calib.HasImuExtrinsic()) {
            if (WriteOneImuCalibXml(sub, calib.ImuRotation(), calib.ImuTranslationMm(), calib.ImuTimeshiftMs(), calib.GyroNoiseDensity(),
                    calib.GyroRandomWalk(), calib.AccelNoiseDensity(), calib.AccelRandomWalk())) {
                RCLCPP_INFO(Logger(), "[NODE] IMU<->left calib xml written to %s", sub.c_str());
            } else {
                RCLCPP_ERROR(Logger(), "[NODE] IMU<->left calib xml partially failed, dir=%s", sub.c_str());
            }
        } else if (imuOnColorOnly) {
            RCLCPP_INFO(Logger(), "[NODE] a %s camera carries the IMU extrinsic against the color camera only; %s holds camera geometry", typeName,
                sub.c_str());
        } else {
            RCLCPP_WARN(Logger(), "[NODE] IMU<->left extrinsic is empty on a %s camera, which should carry it; upload the stereo-channel IMU calibration",
                typeName);
        }

        if (!hasColorLens) {
            RCLCPP_INFO(Logger(), "[NODE] a %s camera has no color lens; xml_rgb not written", typeName);
            return;
        }

        if (!calib.HasColor()) {
            RCLCPP_WARN(Logger(), "[NODE] color calibration is unreadable on a %s camera, which should carry it; xml_rgb not written", typeName);
            return;
        }

        const std::string subRgb = dir + "/xml_rgb";
        std::filesystem::create_directories(subRgb);

        if (calib.HasColorExtrinsic()) {
            if (WriteColorGeometryXml(calib, subRgb)) {
                RCLCPP_INFO(Logger(), "[NODE] color calib xml written to %s (M2 D2 t_P2 iR_2 T_reg R T)", subRgb.c_str());
            } else {
                RCLCPP_ERROR(Logger(), "[NODE] color calib xml partially failed, dir=%s", subRgb.c_str());
            }
        } else {
            RCLCPP_WARN(Logger(), "[NODE] color extrinsic is unusable on a %s camera, which should carry it; color geometry xml not written to %s", typeName,
                subRgb.c_str());
        }

        if (calib.HasColorImuExtrinsic()) {
            if (WriteOneImuCalibXml(subRgb, calib.ColorImuRotation(), calib.ColorImuTranslationMm(), calib.ColorImuTimeshiftMs(),
                    calib.ColorGyroNoiseDensity(), calib.ColorGyroRandomWalk(), calib.ColorAccelNoiseDensity(), calib.ColorAccelRandomWalk())) {
                RCLCPP_INFO(Logger(), "[NODE] IMU<->color calib xml written to %s", subRgb.c_str());
            } else {
                RCLCPP_ERROR(Logger(), "[NODE] IMU<->color calib xml partially failed, dir=%s", subRgb.c_str());
            }
        } else {
            RCLCPP_WARN(Logger(), "[NODE] IMU<->color extrinsic is empty on a %s camera, which should carry it; upload the RGB-channel IMU calibration",
                typeName);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(Logger(), "[NODE] calib xml write error(%s), dir=%s", e.what(), dir.c_str());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("movesense_x95_ros2");

    CameraConfig cfg = LoadConfig(node);

    CameraDriver driver;
    if (!driver.Init(cfg)) {
        RCLCPP_ERROR(Logger(), "[movesense_x95_ros2] CameraDriver.Init failed, exiting");
        return 1;
    }

    StereoCalib calib;
    if (!calib.Init(driver.Camera())) {
        RCLCPP_WARN(Logger(), "[movesense_x95_ros2] no valid stereo extrinsics (camera may be uncalibrated); still publishing images/IMU");
    }

    RCLCPP_INFO(Logger(), "[movesense_x95_ros2] topics: left=%s right=%s depth=%s det=%s imu=%s (stereo mono8)", cfg.cam1Topic.c_str(),
        cfg.cam0Topic.c_str(), cfg.depthTopic.c_str(), cfg.detTopic.c_str(), cfg.imuTopic.c_str());

    StereoFrameQueue frameQueue(static_cast<size_t>(cfg.frameQueueSize));
    ImuQueue imuQueue(static_cast<size_t>(cfg.imuQueueSize));

    StereoPublisher stereoPublisher;
    ImuPublisher imuPublisher;
    StereoReceiver stereoReceiver;
    ImuReceiver imuReceiver;
    TfPublisher tfPublisher;

    const bool colorActive = cfg.enableColor && !driver.IsPassive();
    const bool depthRegistered = cfg.registration == 1 && !driver.IsPassive();

    bool colorCalibOk = false;
    if ((colorActive || depthRegistered || !cfg.imuCalibDir.empty()) && !driver.IsPassive()) {
        colorCalibOk = calib.InitColor(driver.Camera()) && calib.HasColor();
    }

    calib.LogSummary();
    WriteCalibXml(calib, cfg.imuCalibDir, driver.CameraType());

    const bool depthInColorFrame = depthRegistered && colorCalibOk;
    const std::string depthFrameId = depthInColorFrame ? cfg.colorFrameId : cfg.cam0FrameId;

    if (calib.IsValid()) {
        sensor_msgs::msg::CameraInfo depthInfo;
        if (depthInColorFrame) {
            depthInfo =
                MakeCameraInfo(calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy(), cfg.depthWidth, cfg.depthHeight, depthFrameId);
        } else {
            if (depthRegistered) {
                RCLCPP_WARN(Logger(),
                    "[movesense_x95_ros2] registration is on but color calib is invalid; depth camera_info falls back to stereo intrinsics");
            } else if (cfg.registration == -1) {
                RCLCPP_WARN(Logger(), "[movesense_x95_ros2] registration=-1 (camera-side state unknown); depth camera_info uses stereo intrinsics");
            }
            depthInfo = MakeCameraInfo(calib.Fx(), calib.Fy(), calib.Cx(), calib.Cy(), cfg.depthWidth, cfg.depthHeight, depthFrameId);
        }
        stereoPublisher.SetDepthCameraInfo(node, cfg.depthInfoTopic, depthInfo, cfg.frameQueueSize);
    }

    if (colorActive && colorCalibOk) {
        sensor_msgs::msg::CameraInfo colorInfo =
            MakeCameraInfo(calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy(), cfg.colorWidth, cfg.colorHeight, cfg.colorFrameId);
        stereoPublisher.SetColorCameraInfo(node, cfg.colorInfoTopic, colorInfo, cfg.frameQueueSize);
    }

    if (cfg.publishTf) {
        tfPublisher.Publish(node, cfg, calib, colorActive || depthInColorFrame, driver.IsPassive(), driver.CameraType());
    }

    bool ok = true;
    ok = ok &&
         stereoPublisher.Init(node, &frameQueue, cfg.cam0Topic, cfg.cam1Topic, cfg.colorTopic, cfg.depthTopic, cfg.detTopic, cfg.cam0FrameId,
             cfg.cam1FrameId, cfg.colorFrameId, depthFrameId, cfg.detFrameId, cfg.frameQueueSize, cfg.enableStereo, colorActive, cfg.enableDetection);
    ok = ok && stereoReceiver.Init(driver.Camera(), &frameQueue, cfg.frameTimeoutMs);
    if (cfg.enableImu) {
        ok = ok && imuPublisher.Init(node, &imuQueue, cfg.imuTopic, cfg.imuFrameId, cfg.imuQueueSize);
        ok = ok && imuReceiver.Init(driver.Camera(), &imuQueue, cfg.imuPollMs);
    }

    if (!ok) {
        RCLCPP_ERROR(Logger(), "[movesense_x95_ros2] thread Init failed, exiting");

        stereoReceiver.Shutdown();
        imuReceiver.Shutdown();
        stereoPublisher.Shutdown();
        imuPublisher.Shutdown();
        driver.Shutdown();

        return 1;
    }

    rclcpp::spin(node);

    RCLCPP_INFO(Logger(), "[movesense_x95_ros2] shutting down ...");

    std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        std::_Exit(0);
    }).detach();

    stereoReceiver.Shutdown();
    imuReceiver.Shutdown();

    stereoPublisher.Shutdown();
    imuPublisher.Shutdown();

    driver.Shutdown();

    RCLCPP_INFO(Logger(), "[movesense_x95_ros2] exited (frames dropped %llu, IMU dropped %llu)",
        static_cast<unsigned long long>(frameQueue.Dropped()), static_cast<unsigned long long>(imuQueue.Dropped()));

    std::_Exit(0);
}
