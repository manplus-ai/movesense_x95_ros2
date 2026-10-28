// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros2/stereo_calib.h"

#include <movesense/Simou3Camera.h>
#include <rclcpp/rclcpp.hpp>

namespace movesense_x95_ros2 {

static rclcpp::Logger Logger()
{
    return rclcpp::get_logger("StereoCalib");
}

StereoCalib::StereoCalib() {}

StereoCalib::~StereoCalib() {}

bool StereoCalib::Init(movesense::Simou3Camera* cam)
{
    ResetStereo();
    if (cam == nullptr) {
        RCLCPP_ERROR(Logger(), "[StereoCalib] Init null argument");
        return false;
    }

    movesense::calib::StereoCalibration calibration;
    const int result = cam->getStereoCalibData(calibration);
    if (result != movesense::calib::kStereoCalibBytes) {
        RCLCPP_WARN(Logger(), "[StereoCalib] getStereoCalibData failed (ret=%d)", result);
        return false;
    }

    if (!LoadStereoCalibration(calibration)) {
        RCLCPP_WARN(Logger(), "[StereoCalib] stereo geometry is non-finite, degenerate or invalid");
        return false;
    }

    if (!HasImuExtrinsic()) {
        RCLCPP_WARN(Logger(), "[StereoCalib] stereo IMU calibration is unavailable or invalid");
    }

    return true;
}

bool StereoCalib::InitColor(movesense::Simou3Camera* cam)
{
    ResetColor();
    if (cam == nullptr) {
        RCLCPP_ERROR(Logger(), "[StereoCalib] InitColor null argument");
        return false;
    }

    movesense::calib::RgbCalibration calibration;
    const int result = cam->getRGBCalibData(calibration);
    if (result != movesense::calib::kRgbCalibBytes) {
        RCLCPP_WARN(Logger(), "[StereoCalib] getRGBCalibData failed (ret=%d)", result);
        return false;
    }

    if (!LoadColorCalibration(calibration)) {
        RCLCPP_WARN(Logger(), "[StereoCalib] color geometry is non-finite, degenerate or invalid");
        return false;
    }

    if (!HasColorImuExtrinsic()) {
        RCLCPP_WARN(Logger(), "[StereoCalib] color IMU calibration is unavailable or invalid");
    }

    return true;
}

void StereoCalib::LogSummary() const
{
    if (IsValid()) {
        RCLCPP_INFO(Logger(), "[StereoCalib] rectified intrinsics (1280 domain) fx=%.3f fy=%.3f cx=%.3f cy=%.3f baseline=%.4f mm", Fx(), Fy(), Cx(),
            Cy(), BaselineMm());
    } else {
        RCLCPP_WARN(Logger(), "[StereoCalib] stereo geometry unavailable or invalid");
    }

    if (HasImuExtrinsic()) {
        const auto& imu = StereoImuCalibration();
        RCLCPP_INFO(Logger(), "[StereoCalib] IMU to raw left t=[%.4f %.4f %.4f] mm ts=%.4f ms intrinsics=%s", imu.translationMm[0],
            imu.translationMm[1], imu.translationMm[2], imu.timeshiftMs, imu.hasIntrinsics ? "available" : "unavailable");
    }

    if (HasColor()) {
        RCLCPP_INFO(Logger(), "[StereoCalib] color rectified intrinsics (1280 domain) fx=%.3f fy=%.3f cx=%.3f cy=%.3f", ColorFx(), ColorFy(),
            ColorCx(), ColorCy());
    }

    if (HasColorImuExtrinsic()) {
        const auto& imu = ColorImuCalibration();
        RCLCPP_INFO(Logger(), "[StereoCalib] IMU to raw color t=[%.4f %.4f %.4f] mm ts=%.4f ms intrinsics=%s", imu.translationMm[0],
            imu.translationMm[1], imu.translationMm[2], imu.timeshiftMs, imu.hasIntrinsics ? "available" : "unavailable");
    }
}

} // namespace movesense_x95_ros2
