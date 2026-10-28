// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros2/imu_publisher.h"

#include <exception>

namespace movesense_x95_ros2 {

static rclcpp::Logger Logger()
{
    return rclcpp::get_logger("ImuPublisher");
}

ImuPublisher::ImuPublisher() {}

ImuPublisher::~ImuPublisher() {}

bool ImuPublisher::Init(rclcpp::Node::SharedPtr node, ImuQueue* imuQueue, const std::string& imuTopic, const std::string& frameId, int queueSize)
{
    if (node == nullptr || imuQueue == nullptr || queueSize < 1 || imuTopic.empty() || frameId.empty() || m_thread.joinable()) {
        RCLCPP_ERROR(Logger(), "[ImuPublisher] invalid initialization");
        return false;
    }

    m_imuQueue = imuQueue;
    m_clock = node->get_clock();
    m_frameId = frameId;

    try {
        m_imuPub = node->create_publisher<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(queueSize)));
        m_running.store(true);
        m_thread = std::thread(&ImuPublisher::Loop, this);
    } catch (const std::exception& error) {
        m_running.store(false);
        m_imuQueue = nullptr;
        m_imuPub.reset();
        m_clock.reset();
        RCLCPP_ERROR(Logger(), "[ImuPublisher] initialization failed: %s", error.what());
        return false;
    }

    return true;
}

void ImuPublisher::Shutdown()
{
    m_running.store(false);
    if (m_imuQueue != nullptr) {
        m_imuQueue->Stop();
    }

    if (m_thread.joinable()) {
        m_thread.join();
    }
    m_imuQueue = nullptr;
    m_imuPub.reset();
    m_clock.reset();
}

void ImuPublisher::Loop()
{
    while (m_running.load()) {
        ImuSample sample;

        if (!m_imuQueue->Pop(sample)) {
            break;
        }

        sensor_msgs::msg::Imu msg;

        if (sample.tsUs == 0) {
            RCLCPP_ERROR_THROTTLE(Logger(), *m_clock, 1000, "[ImuPublisher] IMU sample has invalid timestamp (tsUs==0)");
            continue;
        }
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(sample.tsUs) * 1000, RCL_ROS_TIME);

        msg.header.frame_id = m_frameId;

        msg.linear_acceleration.x = sample.ax;
        msg.linear_acceleration.y = sample.ay;
        msg.linear_acceleration.z = sample.az;

        msg.angular_velocity.x = sample.gx;
        msg.angular_velocity.y = sample.gy;
        msg.angular_velocity.z = sample.gz;

        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        msg.orientation_covariance[0] = -1.0;

        m_imuPub->publish(msg);
    }
}

} // namespace movesense_x95_ros2
