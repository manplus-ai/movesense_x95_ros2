// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#pragma once

#include "movesense_x95_ros2/common.h"
#include "movesense_x95_ros2/thread_safe_queue.h"

#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <thread>

namespace movesense_x95_ros2 {

class ImuPublisher {
public:
    ImuPublisher();
    ~ImuPublisher();

    bool Init(rclcpp::Node::SharedPtr node, ImuQueue* imuQueue, const std::string& imuTopic, const std::string& frameId, int queueSize);
    void Shutdown();

private:
    void Loop();

    ImuQueue* m_imuQueue = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPub;
    rclcpp::Clock::SharedPtr m_clock;
    std::string m_frameId = "movesense_imu_frame";
    std::atomic<bool> m_running { false };
    std::thread m_thread;
};

} // namespace movesense_x95_ros2
