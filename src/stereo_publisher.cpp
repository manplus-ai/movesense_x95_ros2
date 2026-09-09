// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

#include "movesense_x95_ros2/stereo_publisher.h"

#include "movesense_x95_ros2/seg_result.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

namespace movesense_x95_ros2 {

static rclcpp::Logger Logger()
{
    return rclcpp::get_logger("StereoPublisher");
}

StereoPublisher::StereoPublisher() {}

StereoPublisher::~StereoPublisher() {}

bool StereoPublisher::Init(rclcpp::Node::SharedPtr node, StereoFrameQueue* frameQueue, const std::string& cam0Topic, const std::string& cam1Topic,
    const std::string& colorTopic, const std::string& depthTopic, const std::string& detTopic, const std::string& cam0FrameId,
    const std::string& cam1FrameId, const std::string& colorFrameId, const std::string& depthFrameId, const std::string& detFrameId, int queueSize,
    bool enableStereo, bool enableColor, bool enableDetection)
{
    if (node == nullptr || frameQueue == nullptr) {
        RCLCPP_ERROR(Logger(), "[StereoPublisher] Init null argument");
        return false;
    }

    m_frameQueue = frameQueue;
    m_clock = node->get_clock();
    m_cam0FrameId = cam0FrameId;
    m_cam1FrameId = cam1FrameId;
    m_colorFrameId = colorFrameId;
    m_depthFrameId = depthFrameId;
    m_detFrameId = detFrameId;
    m_stereoEnabled = enableStereo;
    m_colorEnabled = enableColor;
    m_detEnabled = enableDetection;

    const size_t qosDepth = queueSize < 1 ? 1 : static_cast<size_t>(queueSize);
    const auto sensorQos = rclcpp::SensorDataQoS().keep_last(qosDepth);

    if (m_stereoEnabled) {
        m_cam0Pub = node->create_publisher<sensor_msgs::msg::Image>(cam0Topic, sensorQos);
        m_cam1Pub = node->create_publisher<sensor_msgs::msg::Image>(cam1Topic, sensorQos);
    }
    m_depthPub = node->create_publisher<sensor_msgs::msg::Image>(depthTopic, sensorQos);
    if (m_colorEnabled) {
        m_colorPub = node->create_publisher<sensor_msgs::msg::Image>(colorTopic, sensorQos);
    }
    if (m_detEnabled) {
        m_detPub = node->create_publisher<vision_msgs::msg::Detection2DArray>(detTopic, rclcpp::QoS(qosDepth));
    }

    m_running.store(true);
    m_thread = std::thread(&StereoPublisher::Loop, this);

    return true;
}

void StereoPublisher::Shutdown()
{
    m_running.store(false);
    m_frameQueue->Stop();

    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void StereoPublisher::SetDepthCameraInfo(
    rclcpp::Node::SharedPtr node, const std::string& topic, const sensor_msgs::msg::CameraInfo& info, int queueSize)
{
    m_depthInfo = info;
    m_depthInfoPub = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic, rclcpp::QoS(queueSize < 1 ? 1 : static_cast<size_t>(queueSize)));
    m_hasDepthInfo = true;
}

void StereoPublisher::SetColorCameraInfo(
    rclcpp::Node::SharedPtr node, const std::string& topic, const sensor_msgs::msg::CameraInfo& info, int queueSize)
{
    m_colorInfo = info;
    m_colorInfoPub = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic, rclcpp::QoS(queueSize < 1 ? 1 : static_cast<size_t>(queueSize)));
    m_hasColorInfo = true;
}

rclcpp::Time StereoPublisher::StampFromPts(uint64_t ptsUs) const
{
    if (ptsUs == 0) {
        return m_clock->now();
    }

    return rclcpp::Time(static_cast<int64_t>(ptsUs) * 1000, RCL_ROS_TIME);
}

void StereoPublisher::PublishOne(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub, const PlaneData& img, const std::string& frameId)
{
    if (!img.valid) {
        return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = StampFromPts(img.ptsUs);
    msg.header.frame_id = frameId;
    msg.is_bigendian = 0;

    if (img.format == static_cast<uint8_t>(1)) {
        const size_t needNv21 = static_cast<size_t>(img.width) * img.height * 3 / 2;
        if (img.data.size() < needNv21) {
            return;
        }

        cv::Mat nv21(img.height * 3 / 2, img.width, CV_8UC1, const_cast<uint8_t*>(img.data.data()));
        cv::Mat bgr;
        cv::cvtColor(nv21, bgr, cv::COLOR_YUV2BGR_NV21);

        msg.height = bgr.rows;
        msg.width = bgr.cols;
        msg.encoding = "bgr8";
        msg.step = bgr.cols * 3;
        msg.data.assign(bgr.data, bgr.data + static_cast<size_t>(bgr.cols) * bgr.rows * 3);
    } else {
        const size_t needY = static_cast<size_t>(img.width) * img.height;
        if (img.data.size() < needY) {
            return;
        }

        msg.height = img.height;
        msg.width = img.width;
        msg.encoding = "mono8";
        msg.step = img.width;
        msg.data.assign(img.data.begin(), img.data.begin() + needY);
    }

    pub->publish(msg);
}

void StereoPublisher::PublishColor(const PlaneData& color)
{
    if (!m_colorEnabled || !color.valid) {
        return;
    }

    const size_t needNv21 = static_cast<size_t>(color.width) * color.height * 3 / 2;

    if (color.data.size() < needNv21) {
        return;
    }

    cv::Mat nv21(color.height * 3 / 2, color.width, CV_8UC1, const_cast<uint8_t*>(color.data.data()));
    cv::Mat bgr;
    cv::cvtColor(nv21, bgr, cv::COLOR_YUV2BGR_NV21);

    sensor_msgs::msg::Image msg;
    msg.header.stamp = StampFromPts(color.ptsUs);
    msg.header.frame_id = m_colorFrameId;
    msg.height = bgr.rows;
    msg.width = bgr.cols;
    msg.is_bigendian = 0;
    msg.encoding = "bgr8";
    msg.step = bgr.cols * 3;
    msg.data.assign(bgr.data, bgr.data + static_cast<size_t>(bgr.cols) * bgr.rows * 3);

    m_colorPub->publish(msg);

    if (m_hasColorInfo) {
        m_colorInfo.header.stamp = msg.header.stamp;
        m_colorInfoPub->publish(m_colorInfo);
    }
}

void StereoPublisher::PublishDepth(const PlaneData& depth)
{
    if (!depth.valid) {
        return;
    }

    const size_t need = static_cast<size_t>(depth.width) * depth.height * 2;

    if (depth.data.size() < need) {
        return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = StampFromPts(depth.ptsUs);
    msg.header.frame_id = m_depthFrameId;
    msg.height = depth.height;
    msg.width = depth.width;
    msg.is_bigendian = 0;
    msg.encoding = "16UC1";
    msg.step = depth.width * 2;
    msg.data.assign(depth.data.begin(), depth.data.begin() + need);

    m_depthPub->publish(msg);

    if (m_hasDepthInfo) {
        m_depthInfo.header.stamp = msg.header.stamp;
        m_depthInfoPub->publish(m_depthInfo);
    }
}

void StereoPublisher::PublishDetections(const PlaneData& seg)
{
    if (!m_detEnabled || !seg.valid) {
        return;
    }

    const int bytes = static_cast<int>(seg.data.size());

    if (!SegValid(seg.data.data(), bytes)) {
        return;
    }

    const SegHeader* r = reinterpret_cast<const SegHeader*>(seg.data.data());
    const SegObject* dets = r->Detections();

    vision_msgs::msg::Detection2DArray arr;
    arr.header.stamp = StampFromPts(seg.ptsUs);
    arr.header.frame_id = m_detFrameId;
    arr.detections.reserve(r->detectionCount);

    for (uint16_t i = 0; i < r->detectionCount; ++i) {
        const SegObject& o = dets[i];

        vision_msgs::msg::Detection2D d;
        d.header = arr.header;
        d.bbox.center.position.x = (o.x1 + o.x2) * 0.5;
        d.bbox.center.position.y = (o.y1 + o.y2) * 0.5;
        d.bbox.size_x = o.x2 - o.x1;
        d.bbox.size_y = o.y2 - o.y1;

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = std::to_string(o.classId);
        hyp.hypothesis.score = o.score;
        d.results.push_back(hyp);

        arr.detections.push_back(std::move(d));
    }

    m_detPub->publish(arr);
}

void StereoPublisher::Loop()
{
    while (m_running.load()) {
        StereoFrame payload;

        if (!m_frameQueue->Pop(payload)) {
            break;
        }

        if (m_stereoEnabled) {
            PublishOne(m_cam0Pub, payload.right, m_cam0FrameId);
            PublishOne(m_cam1Pub, payload.left, m_cam1FrameId);
        }
        PublishColor(payload.color);
        PublishDepth(payload.depth);
        PublishDetections(payload.seg);
    }
}

} // namespace movesense_x95_ros2
