# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Humanplus Intelligent Robotics Technology Co.,Ltd. All rights reserved.

# Defaults below target active (A / AP) cameras.
# On a passive (P) camera there is no RGB lens and no DOE projector:
# set enable_color:=false and enable_stereo:=true (the rectified
# stereo pair is P's image source); depth->color registration and
# doe_power have no effect (depth stays in the stereo frame).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# name, default, python type (str params pass through as-is)
ARGS = [
    # connection / time
    ('camera_ip', '192.168.1.70', str),
    ('align_time_on_start', 'false', bool),   # run one NTP time alignment (alignTimeToHost) on startup
    ('first_frame_wait_ms', '5000', int),     # block Init until the first frame arrives, then read calibration (0 = do not wait)
    # frame / resolution: each stream is 1280x960 or 640x480 only.
    # Color is independent of the stereo/depth pipeline; downsample_mode only affects stereo and depth:
    #   stereo 640  + depth 640  -> pre or post, both valid
    #   stereo 1280 + depth 640  -> post only (pre would shrink the pair too)
    #   stereo 640  + depth 1280 -> unsupported
    #   stereo 1280 + depth 1280 -> nothing is downsampled, the mode is ignored
    ('downsample_mode', '0', int),            # 0 = pre (downsample, then match) / 1 = post (match, then downsample) / -1 = auto
    ('depth_fps', '25', int),                 # frame rate 5~25
    ('stereo_width', '640', int),
    ('stereo_height', '480', int),
    ('depth_width', '640', int),
    ('depth_height', '480', int),
    ('color_width', '640', int),              # A/AP only
    ('color_height', '480', int),             # A/AP only
    # stream subscription
    ('enable_stereo', 'false', bool),         # left/right rectified images
    ('enable_depth', 'true', bool),
    ('enable_color', 'true', bool),           # A/AP only, set false on P
    ('enable_imu', 'true', bool),
    ('enable_detection', 'false', bool),      # AI detection (YOLO seg)
    # stereo exposure / gain
    ('stereo_auto_expo', 'false', bool),
    ('stereo_exposure_us', '5000', int),
    ('stereo_gain_x', '1', int),              # gain multiplier (1x..16x)
    ('stereo_max_exposure_us', '65535', int), # -1 = keep camera setting
    ('stereo_min_exposure_us', '100', int),
    ('stereo_max_gain_x', '16', int),
    ('stereo_min_gain_x', '1', int),
    # color exposure / gain (effective only with enable_color)
    ('color_auto_expo', 'false', bool),
    ('color_exposure_us', '5000', int),
    ('color_gain_x', '1', int),
    ('color_max_exposure_us', '65535', int),
    ('color_min_exposure_us', '100', int),
    ('color_max_gain_x', '16', int),
    ('color_min_gain_x', '1', int),
    # projector / depth options
    ('doe_power', '255', int),                # DOE 0..255 (0 = off); -1 = keep; A/AP only
    ('registration', '1', int),               # 1 = on / 0 = off / -1 = keep; A/AP only
    # imu calib export
    ('imu_calib_dir', '', str),               # output dir for stereo-IMU extrinsics XMLs (empty = skip)
    # topics
    ('cam0_topic', '/movesense/right/image_rect_raw', str),
    ('cam1_topic', '/movesense/left/image_rect_raw', str),
    ('color_topic', '/movesense/color/image_rect_color', str),
    ('color_info_topic', '/movesense/color/camera_info', str),
    ('depth_topic', '/movesense/depth/image_rect_raw', str),
    ('depth_info_topic', '/movesense/depth/camera_info', str),
    ('det_topic', '/movesense/detections', str),
    ('imu_topic', '/movesense/imu', str),
    # tf / frame ids
    ('publish_tf', 'true', bool),                                  # broadcast the static TF tree (turn off when a URDF provides it)
    ('base_frame_id', 'movesense_link', str),                      # tree root
    ('cam0_frame_id', 'movesense_infra2_optical_frame', str),      # right eye
    ('cam1_frame_id', 'movesense_infra1_optical_frame', str),      # left eye = reference frame of all calibration
    ('color_frame_id', 'movesense_color_optical_frame', str),
    ('det_frame_id', 'movesense_color_optical_frame', str),
    ('imu_frame_id', 'movesense_imu_frame', str),
    # roi: crop window per stream, cut on the camera before sending (depth cannot be cropped).
    # Coordinates are 0-based pixels in that stream's current output size (after downsampling):
    #   window = [x1,x2) x [y1,y2), x1<x2, y1<y2, at least 16x16, all four values even.
    # Needs camera firmware with ROI support; an enabled ROI the camera rejects stops the node.
    ('roi_left_raw_enable', 'false', bool),
    ('roi_left_raw_x1', '0', int),
    ('roi_left_raw_y1', '0', int),
    ('roi_left_raw_x2', '0', int),
    ('roi_left_raw_y2', '0', int),
    ('roi_right_raw_enable', 'false', bool),
    ('roi_right_raw_x1', '0', int),
    ('roi_right_raw_y1', '0', int),
    ('roi_right_raw_x2', '0', int),
    ('roi_right_raw_y2', '0', int),
    ('roi_color_raw_enable', 'false', bool),
    ('roi_color_raw_x1', '0', int),
    ('roi_color_raw_y1', '0', int),
    ('roi_color_raw_x2', '0', int),
    ('roi_color_raw_y2', '0', int),
    ('roi_left_rect_enable', 'false', bool),
    ('roi_left_rect_x1', '0', int),
    ('roi_left_rect_y1', '0', int),
    ('roi_left_rect_x2', '0', int),
    ('roi_left_rect_y2', '0', int),
    ('roi_right_rect_enable', 'false', bool),
    ('roi_right_rect_x1', '0', int),
    ('roi_right_rect_y1', '0', int),
    ('roi_right_rect_x2', '0', int),
    ('roi_right_rect_y2', '0', int),
    ('roi_color_rect_enable', 'false', bool),
    ('roi_color_rect_x1', '0', int),
    ('roi_color_rect_y1', '0', int),
    ('roi_color_rect_x2', '0', int),
    ('roi_color_rect_y2', '0', int),
]


def generate_launch_description():
    declares = [DeclareLaunchArgument(name, default_value=default) for name, default, _ in ARGS]

    params = {}
    for name, _, ptype in ARGS:
        if ptype is str:
            params[name] = LaunchConfiguration(name)
        else:
            params[name] = ParameterValue(LaunchConfiguration(name), value_type=ptype)

    node = Node(
        package='movesense_x95_ros2',
        executable='movesense_x95_ros2',
        name='movesense_x95_ros2',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription(declares + [node])
