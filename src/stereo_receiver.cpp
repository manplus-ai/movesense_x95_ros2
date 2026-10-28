// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros2/stereo_receiver.h"

#include <rclcpp/rclcpp.hpp>

namespace movesense_x95_ros2 {

StereoReceiver::StereoReceiver() {}

StereoReceiver::~StereoReceiver() {}

bool StereoReceiver::Init(movesense::Simou3Camera* cam, StereoFrameQueue* queue, int timeoutMs, uint64_t minimumTimestampUs)
{
    return FrameReceiver::Init(
        cam, queue, timeoutMs,
        [](movesense_x95::LogLevel level, const std::string& message) {
            if (level == movesense_x95::LogLevel::Error) {
                RCLCPP_ERROR(rclcpp::get_logger("StereoReceiver"), "%s", message.c_str());
            } else if (level == movesense_x95::LogLevel::Warning) {
                RCLCPP_WARN(rclcpp::get_logger("StereoReceiver"), "%s", message.c_str());
            } else {
                RCLCPP_INFO(rclcpp::get_logger("StereoReceiver"), "%s", message.c_str());
            }
        },
        minimumTimestampUs);
}

} // namespace movesense_x95_ros2
