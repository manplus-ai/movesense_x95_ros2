// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros2/camera_driver.h"

#include <rclcpp/rclcpp.hpp>

namespace movesense_x95_ros2 {

CameraDriver::CameraDriver() {}

CameraDriver::~CameraDriver() {}

bool CameraDriver::Init(const CameraConfig& cfg)
{
    return CameraSession::Init(cfg, [](movesense_x95::LogLevel level, const std::string& message) {
        if (level == movesense_x95::LogLevel::Error) {
            RCLCPP_ERROR(rclcpp::get_logger("CameraDriver"), "%s", message.c_str());
        } else if (level == movesense_x95::LogLevel::Warning) {
            RCLCPP_WARN(rclcpp::get_logger("CameraDriver"), "%s", message.c_str());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("CameraDriver"), "%s", message.c_str());
        }
    });
}

} // namespace movesense_x95_ros2
