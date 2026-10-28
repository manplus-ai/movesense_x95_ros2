// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95/calibration_export.h"
#include "movesense_x95/camera_config.h"
#include "movesense_x95/session_supervisor.h"
#include "movesense_x95/thread_safe_queue.h"
#include "movesense_x95_ros2/camera_driver.h"
#include "movesense_x95_ros2/imu_publisher.h"
#include "movesense_x95_ros2/imu_receiver.h"
#include "movesense_x95_ros2/stereo_calib.h"
#include "movesense_x95_ros2/stereo_publisher.h"
#include "movesense_x95_ros2/stereo_receiver.h"
#include "movesense_x95_ros2/tf_publisher.h"

#include <chrono>
#include <iostream>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stdexcept>
#include <string>
#include <thread>

using namespace movesense_x95_ros2;
using namespace movesense_x95;

template<typename T>
static T ReadParameter(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& value)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node->declare_parameter<T>(name, value, descriptor);
}

static CameraConfig LoadConfig(const rclcpp::Node::SharedPtr& node)
{
    CameraConfig cfg;
    cfg.cameraIp = ReadParameter<std::string>(node, "camera_ip", cfg.cameraIp);
    cfg.alignTimeOnStart = ReadParameter<bool>(node, "align_time_on_start", cfg.alignTimeOnStart);
    cfg.connectionProbe = ReadParameter<bool>(node, "connection_probe", cfg.connectionProbe);
    cfg.triggerMode = ReadParameter<int>(node, "trigger_mode", cfg.triggerMode);
    cfg.firstFrameWaitMs = ReadParameter<int>(node, "first_frame_wait_ms", cfg.firstFrameWaitMs);
    cfg.fps = ReadParameter<int>(node, "depth_fps", cfg.fps);
    cfg.stereoWidth = ReadParameter<int>(node, "stereo_width", cfg.stereoWidth);
    cfg.stereoHeight = ReadParameter<int>(node, "stereo_height", cfg.stereoHeight);
    cfg.downsampleMode = ReadParameter<int>(node, "downsample_mode", cfg.downsampleMode);
    cfg.depthWidth = ReadParameter<int>(node, "depth_width", cfg.depthWidth);
    cfg.depthHeight = ReadParameter<int>(node, "depth_height", cfg.depthHeight);
    cfg.colorWidth = ReadParameter<int>(node, "color_width", cfg.colorWidth);
    cfg.colorHeight = ReadParameter<int>(node, "color_height", cfg.colorHeight);
    cfg.enableStereo = ReadParameter<bool>(node, "enable_stereo", cfg.enableStereo);
    cfg.enableDepth = ReadParameter<bool>(node, "enable_depth", cfg.enableDepth);
    cfg.enableColor = ReadParameter<bool>(node, "enable_color", cfg.enableColor);
    cfg.enableImu = ReadParameter<bool>(node, "enable_imu", cfg.enableImu);
    cfg.enableDetection = ReadParameter<bool>(node, "enable_detection", cfg.enableDetection);
    cfg.stereoAutoExpo = ReadParameter<bool>(node, "stereo_auto_expo", cfg.stereoAutoExpo);
    cfg.stereoExposureUs = ReadParameter<int>(node, "stereo_exposure_us", cfg.stereoExposureUs);
    cfg.stereoGainX = ReadParameter<int>(node, "stereo_gain_x", cfg.stereoGainX);
    cfg.stereoMaxExposureUs = ReadParameter<int>(node, "stereo_max_exposure_us", cfg.stereoMaxExposureUs);
    cfg.stereoMinExposureUs = ReadParameter<int>(node, "stereo_min_exposure_us", cfg.stereoMinExposureUs);
    cfg.stereoMaxGainX = ReadParameter<int>(node, "stereo_max_gain_x", cfg.stereoMaxGainX);
    cfg.stereoMinGainX = ReadParameter<int>(node, "stereo_min_gain_x", cfg.stereoMinGainX);
    cfg.colorAutoExpo = ReadParameter<bool>(node, "color_auto_expo", cfg.colorAutoExpo);
    cfg.colorExposureUs = ReadParameter<int>(node, "color_exposure_us", cfg.colorExposureUs);
    cfg.colorGainX = ReadParameter<int>(node, "color_gain_x", cfg.colorGainX);
    cfg.colorMaxExposureUs = ReadParameter<int>(node, "color_max_exposure_us", cfg.colorMaxExposureUs);
    cfg.colorMinExposureUs = ReadParameter<int>(node, "color_min_exposure_us", cfg.colorMinExposureUs);
    cfg.colorMaxGainX = ReadParameter<int>(node, "color_max_gain_x", cfg.colorMaxGainX);
    cfg.colorMinGainX = ReadParameter<int>(node, "color_min_gain_x", cfg.colorMinGainX);
    cfg.doePower = ReadParameter<int>(node, "doe_power", cfg.doePower);
    cfg.registration = ReadParameter<int>(node, "registration", cfg.registration);
    cfg.imuCalibDir = ReadParameter<std::string>(node, "imu_calib_dir", cfg.imuCalibDir);
    cfg.cam0Topic = ReadParameter<std::string>(node, "cam0_topic", cfg.cam0Topic);
    cfg.cam1Topic = ReadParameter<std::string>(node, "cam1_topic", cfg.cam1Topic);
    cfg.colorTopic = ReadParameter<std::string>(node, "color_topic", cfg.colorTopic);
    cfg.colorInfoTopic = ReadParameter<std::string>(node, "color_info_topic", cfg.colorInfoTopic);
    cfg.depthTopic = ReadParameter<std::string>(node, "depth_topic", cfg.depthTopic);
    cfg.depthInfoTopic = ReadParameter<std::string>(node, "depth_info_topic", cfg.depthInfoTopic);
    cfg.detTopic = ReadParameter<std::string>(node, "det_topic", cfg.detTopic);
    cfg.imuTopic = ReadParameter<std::string>(node, "imu_topic", cfg.imuTopic);
    cfg.publishTf = ReadParameter<bool>(node, "publish_tf", cfg.publishTf);
    cfg.baseFrameId = ReadParameter<std::string>(node, "base_frame_id", cfg.baseFrameId);
    cfg.cam0FrameId = ReadParameter<std::string>(node, "cam0_frame_id", cfg.cam0FrameId);
    cfg.cam1FrameId = ReadParameter<std::string>(node, "cam1_frame_id", cfg.cam1FrameId);
    cfg.colorFrameId = ReadParameter<std::string>(node, "color_frame_id", cfg.colorFrameId);
    cfg.detFrameId = ReadParameter<std::string>(node, "det_frame_id", cfg.detFrameId);
    cfg.imuFrameId = ReadParameter<std::string>(node, "imu_frame_id", cfg.imuFrameId);

    for (int i = 0; i < kRoiStreamCount; ++i) {
        const std::string prefix = std::string("roi_") + RoiStreamName(i) + "_";
        RoiConfig& roi = cfg.roi[i];
        roi.enable = ReadParameter<bool>(node, prefix + "enable", roi.enable);
        roi.x1 = ReadParameter<int>(node, prefix + "x1", roi.x1);
        roi.y1 = ReadParameter<int>(node, prefix + "y1", roi.y1);
        roi.x2 = ReadParameter<int>(node, prefix + "x2", roi.x2);
        roi.y2 = ReadParameter<int>(node, prefix + "y2", roi.y2);
    }
    return cfg;
}

static sensor_msgs::msg::CameraInfo MakeCameraInfo(double fx, double fy, double cx, double cy, int width, int height, const std::string& frameId)
{
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frameId;
    info.width = static_cast<uint32_t>(width);
    info.height = static_cast<uint32_t>(height);
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);

    const double scaleX = static_cast<double>(width) / 1280.0;
    const double scaleY = static_cast<double>(height) / 960.0;
    info.k[0] = fx * scaleX;
    info.k[2] = cx * scaleX;
    info.k[4] = fy * scaleY;
    info.k[5] = cy * scaleY;
    info.k[8] = 1.0;
    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;
    info.p[0] = info.k[0];
    info.p[2] = info.k[2];
    info.p[5] = info.k[4];
    info.p[6] = info.k[5];
    info.p[10] = 1.0;
    return info;
}

static SessionResult RunCameraSession(const rclcpp::Node::SharedPtr& node, const CameraConfig& cfg, uint64_t attemptId)
{
    CameraDriver driver;
    StereoCalib calib;
    StereoFrameQueue frameQueue(static_cast<size_t>(cfg.frameQueueSize));
    ImuQueue imuQueue(static_cast<size_t>(cfg.imuQueueSize));
    StereoPublisher stereoPublisher;
    ImuPublisher imuPublisher;
    StereoReceiver stereoReceiver;
    ImuReceiver imuReceiver;
    TfPublisher tfPublisher;
    SessionResult result = SessionResult::Retry;

    try {
        if (!driver.Init(cfg)) {
            throw std::runtime_error("Camera initialization failed");
        }
        if (!calib.Init(driver.Camera())) {
            RCLCPP_WARN(node->get_logger(), "Stereo calibration is invalid; stereo CameraInfo/TF geometry is unavailable");
        }

        const bool colorActive = cfg.enableColor && !driver.IsPassive();
        const bool depthRegistered = cfg.enableDepth && cfg.registration == 1 && !driver.IsPassive();
        const bool needColorCalibration = colorActive || depthRegistered || cfg.enableDetection ||
                                          (cfg.enableImu && (driver.CameraType() == 0 || !calib.HasImuExtrinsic())) || !cfg.imuCalibDir.empty();
        bool colorCalibOk = false;
        if (needColorCalibration && !driver.IsPassive()) {
            colorCalibOk = calib.InitColor(driver.Camera()) && calib.HasColor();
        }

        calib.LogSummary();
        std::string exportError;
        if (!WriteCalibrationXml(calib, cfg.imuCalibDir, driver.CameraType(), exportError)) {
            throw std::runtime_error("Calibration XML export failed: " + exportError);
        }
        if (depthRegistered && (!colorCalibOk || !calib.HasColorExtrinsic())) {
            throw std::runtime_error("Registered depth requires valid RGB calibration");
        }
        if (!cfg.imuCalibDir.empty()) {
            RCLCPP_INFO(node->get_logger(), "Calibration XML written to %s", cfg.imuCalibDir.c_str());
        }

        const std::string detFrameId =
            cfg.detFrameId.empty() || cfg.detFrameId == "auto" ? (driver.IsPassive() ? cfg.cam0FrameId : cfg.colorFrameId) : cfg.detFrameId;
        const std::string depthFrameId = depthRegistered ? cfg.colorFrameId : cfg.cam0FrameId;
        stereoPublisher.SetImageGeometry(cfg, driver.IsPassive());
        if (cfg.enableDepth && !stereoPublisher.SetDepthUnitMm(driver.DepthUnitMm())) {
            throw std::runtime_error("Invalid depth unit");
        }
        if (cfg.enableDepth && (depthRegistered || calib.IsValid())) {
            const sensor_msgs::msg::CameraInfo depthInfo =
                depthRegistered ? MakeCameraInfo(calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy(), cfg.depthWidth, cfg.depthHeight,
                                      depthFrameId)
                                : MakeCameraInfo(calib.Fx(), calib.Fy(), calib.Cx(), calib.Cy(), cfg.depthWidth, cfg.depthHeight, depthFrameId);
            stereoPublisher.SetDepthCameraInfo(node, cfg.depthInfoTopic, depthInfo, cfg.frameQueueSize);
        }
        if (colorActive && colorCalibOk) {
            const sensor_msgs::msg::CameraInfo colorInfo =
                MakeCameraInfo(calib.ColorFx(), calib.ColorFy(), calib.ColorCx(), calib.ColorCy(), cfg.colorWidth, cfg.colorHeight, cfg.colorFrameId);
            stereoPublisher.SetColorCameraInfo(node, cfg.colorInfoTopic, colorInfo, cfg.frameQueueSize);
        }
        if (cfg.publishTf) {
            bool imuOnColor = false;
            const bool imuCalibrationAvailable = calib.SelectImuCalibration(driver.CameraType(), imuOnColor);
            const bool colorFrameUsed = colorActive || depthRegistered || (!driver.IsPassive() && cfg.enableDetection) ||
                                        (cfg.enableImu && imuCalibrationAvailable && imuOnColor);
            tfPublisher.Publish(node, cfg, calib, colorFrameUsed, driver.IsPassive(), driver.CameraType());
        }

        if (!stereoPublisher.Init(node, &frameQueue, cfg.cam0Topic, cfg.cam1Topic, cfg.colorTopic, cfg.depthTopic, cfg.detTopic, cfg.cam0FrameId,
                cfg.cam1FrameId, cfg.colorFrameId, depthFrameId, detFrameId, cfg.frameQueueSize, cfg.enableStereo, colorActive, cfg.enableDetection,
                cfg.enableDepth) ||
            !stereoReceiver.Init(driver.Camera(), &frameQueue, cfg.frameTimeoutMs, driver.MinimumTimestampUs())) {
            throw std::runtime_error("Image thread initialization failed");
        }
        if (cfg.enableImu && (!imuPublisher.Init(node, &imuQueue, cfg.imuTopic, cfg.imuFrameId, cfg.imuQueueSize) ||
                                 !imuReceiver.Init(driver.Camera(), &imuQueue, cfg.imuPollMs, driver.MinimumTimestampUs()))) {
            throw std::runtime_error("IMU thread initialization failed");
        }

        RCLCPP_INFO(node->get_logger(), "Camera session %llu is publishing", static_cast<unsigned long long>(attemptId));
        while (rclcpp::ok() && !driver.Failed() && !stereoReceiver.Failed() && !imuReceiver.Failed()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        result = rclcpp::ok() ? SessionResult::Retry : SessionResult::Stopped;
        if (result == SessionResult::Retry) {
            RCLCPP_WARN(node->get_logger(), "Camera connection was lost; stopping this session before reconnecting");
        }
    } catch (const std::exception& exception) {
        if (rclcpp::ok()) {
            RCLCPP_WARN(node->get_logger(), "Camera session %llu unavailable: %s", static_cast<unsigned long long>(attemptId), exception.what());
        }
        result = rclcpp::ok() ? SessionResult::Retry : SessionResult::Stopped;
    } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Unexpected camera session failure");
        result = SessionResult::Fatal;
    }

    const bool connectionLost = driver.Failed() || stereoReceiver.Failed() || imuReceiver.Failed();
    stereoPublisher.Shutdown();
    imuPublisher.Shutdown();
    stereoReceiver.Shutdown();
    imuReceiver.Shutdown();
    driver.Shutdown(connectionLost);
    calib.Shutdown();

    RCLCPP_INFO(node->get_logger(), "Camera session %llu stopped (frames dropped %llu, IMU dropped %llu)", static_cast<unsigned long long>(attemptId),
        static_cast<unsigned long long>(frameQueue.Dropped()), static_cast<unsigned long long>(imuQueue.Dropped()));
    return result;
}

int main(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--version") {
        std::cout << MOVESENSE_X95_RELEASE_VERSION << std::endl;
        return 0;
    }
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("movesense_x95_ros2");

    int exitCode = 1;
    SessionSupervisor supervisor;
    try {
        const CameraConfig cfg = LoadConfig(node);
        RCLCPP_INFO(node->get_logger(), "MoveSense X95 ROS release %s", MOVESENSE_X95_RELEASE_VERSION);
        if (node->get_parameter("use_sim_time").as_bool()) {
            throw std::runtime_error("Live camera timestamps require use_sim_time=false");
        }
        CameraDriver validator;
        if (!validator.ValidateConfiguration(cfg)) {
            throw std::runtime_error("Invalid camera configuration");
        }
        const auto runSession = [&node, &cfg](uint64_t attemptId) { return RunCameraSession(node, cfg, attemptId); };
        const auto running = []() { return rclcpp::ok(); };
        const auto spin = [&node]() { rclcpp::spin_some(node); };
        const auto logger = [node](LogLevel level, const std::string& message) {
            if (level == LogLevel::Error) {
                RCLCPP_ERROR(node->get_logger(), "%s", message.c_str());
            } else if (level == LogLevel::Warning) {
                RCLCPP_WARN(node->get_logger(), "%s", message.c_str());
            } else {
                RCLCPP_INFO(node->get_logger(), "%s", message.c_str());
            }
        };
        if (!supervisor.Init(runSession, running, spin, logger, 1000)) {
            throw std::runtime_error("Camera session supervisor initialization failed");
        }
        exitCode = supervisor.Run();
    } catch (const std::exception& exception) {
        RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
    }
    supervisor.Shutdown();
    rclcpp::shutdown();
    return exitCode;
}
