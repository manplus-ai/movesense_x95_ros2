# MoveSense X95 ROS 2 Driver (movesense_x95_ros2)

> **Release: 0.5.2.20261028 — early release.** Topics and parameters may change before 1.0.

The release identifier combines SDK 0.5.2 and the release date. ROS requires a three-part package version, so package.xml and the CMake project version are 0.5.2; VERSION, the package export metadata and the executable's --version output retain the full 0.5.2.20261028 identifier.

ROS 2 Humble driver for X95 A / AP / P cameras, using **MoveSense X95 SDK 0.5.2 or newer**. This release was compiled against SDK 0.5.2 and tested with an X95 AP camera; the tested scope is listed below.

## Requirements and build

| Item | Requirement |
| --- | --- |
| OS / ROS | Ubuntu 22.04, ROS 2 Humble, x86-64 |
| Language | C++17 |
| Dependencies | OpenCV, ament_cmake, rclcpp, rcl_interfaces, sensor_msgs, std_msgs, vision_msgs, geometry_msgs, tf2, tf2_ros |
| Camera network | TCP 5001 / 5002; UDP 5000 for host time alignment; UDP 5004 for the optional online probe |

Install the [SDK](https://github.com/manplus-ai/MoveSense_X95_SDK) first:

```bash
cmake -S MoveSense_X95_SDK -B sdk-build -DCMAKE_BUILD_TYPE=Release
cmake --build sdk-build --parallel 4
sudo cmake --install sdk-build
sudo ldconfig
```

Place this repository in the ROS workspace's `src` directory, install the ROS dependencies with rosdep, then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select movesense_x95_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

CMake requires `find_package(MoveSense_X95_SDK 0.5.2 REQUIRED)` and links `MoveSense::X95_SDK`. For a custom SDK prefix, pass `-DMoveSense_X95_SDK_DIR=<prefix>/lib/cmake/MoveSense_X95_SDK` to CMake and make its shared library available to the runtime loader. The node, launch, optional Fast DDS profile, README files and VERSION are installed into the package prefix.

SDK headers and libraries are supplied by the installed SDK package and are not copied into this repository. The imported CMake target provides the SDK include and link paths. The local `include/movesense_x95/` directory contains this driver's shared implementation and must remain in the repository.

## Run

```bash
source install/setup.bash
ros2 launch movesense_x95_ros2 movesense_x95.launch.py camera_ip:=192.168.1.70
```

Defaults select RGB + depth + raw IMU, pre-downsampling, 640×480 and a requested 25 FPS. The effective FPS is read back and logged; available throughput depends on firmware, enabled streams and bandwidth.

P cameras have no separate RGB lens or DOE. Use:

```bash
ros2 launch movesense_x95_ros2 movesense_x95.launch.py enable_color:=false enable_stereo:=true registration:=0
```

For full-resolution stereo with half-resolution depth, select post-downsampling:

```bash
ros2 launch movesense_x95_ros2 movesense_x95.launch.py enable_stereo:=true stereo_width:=1280 stereo_height:=960 downsample_mode:=1
```

## Optional Fast DDS profile for WSL mirrored networking

Some WSL mirrored-network environments drop fragmented UDP datagrams even though small packets and endpoint discovery still work. This can cause delayed discovery or no topic data. The optional installed profile config/fastdds_udp1400_shm.xml limits UDP payloads to 1400 bytes and keeps shared-memory transport for local image traffic. It does not change the camera TCP protocol or system networking, and is not enabled automatically.

For this environment, set the following in both the driver and subscriber shells before starting either process:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix movesense_x95_ros2)/share/movesense_x95_ros2/config/fastdds_udp1400_shm.xml"
```

Both processes must use the same `ROS_DOMAIN_ID`; leaving it unset on both uses the default domain 0. Keep `ROS_LOCALHOST_ONLY=0` when using this profile so its transport selection remains effective. The profile keeps the normal interface selection and does not hardcode a camera IP, DDS domain or peer port. Systems without this fragmentation problem can use their existing middleware configuration.

## Topics and units

The default topic prefix is `/movesense`; topic and frame names can be overridden.

| Topic suffix | Message | Meaning |
| --- | --- | --- |
| `left/image_rect_raw`, `right/image_rect_raw` | Image | Rectified stereo; mono8 on A/AP, bgr8 on P |
| `color/image_rect_color`, `color/camera_info` | Image / CameraInfo | Rectified RGB and matching intrinsics; A/AP only |
| `depth/image_rect_raw`, `depth/camera_info` | Image / CameraInfo | 16UC1 in **millimetres**, matching the selected registration frame |
| `imu` | Imu | Raw specific-force measurements in m/s² and angular velocity in rad/s; no orientation |
| `detections` | Detection2DArray | Boxes in the corresponding published image pixel coordinates |

Disabled streams are not advertised. Left/right CameraInfo topics are not implemented; their full camera geometry is available through XML export.
Images and IMU use SensorDataQoS (best effort). CameraInfo and detections use reliable QoS; subscriptions must use compatible QoS.

The SDK depth unit is queried before streaming. Counts are rounded to the nearest integer millimetre; zero remains invalid, and distances above 65535 mm become zero. Submillimetre precision is not preserved by this 16UC1 topic.

`registration=1` selects RGB-view depth and RGB intrinsics; `0` selects the right stereo view. Registered depth requires valid RGB calibration. The SDK has no registration read-back API: `registration=-1` is accepted only when depth is disabled. P-camera depth always uses the stereo view.

## Parameters

Camera parameters declared by this driver apply at startup and are read-only, so ros2 param set cannot silently change only the parameter server. Restart the node to apply changes.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `camera_ip` | 192.168.1.70 | Camera IP |
| `align_time_on_start` | true | Align SDK timestamps after the first data establishes the camera time base; failure stops initialization |
| `connection_probe` | true | Probe UDP 5004 every 2 seconds; reconnect after 3 consecutive failures |
| `trigger_mode` | 0 | 0: automatic; 2: external; -1: preserve and query current mode |
| `first_frame_wait_ms` | 5000 | Startup image wait, 1…60000 ms |
| `depth_fps` | 25 | Requested 5…25 FPS; effective value logged |
| `stereo_width/height` | 640 / 480 | 640×480 or 1280×960 |
| `depth_width/height` | 640 / 480 | 640×480 or 1280×960 |
| `color_width/height` | 640 / 480 | Independent RGB output resolution |
| `downsample_mode` | 0 | 0: pre; 1: post; -1: choose from requested geometry |
| `enable_stereo` | false | Left/right rectified images |
| `enable_depth/color/imu` | true | Stream subscriptions |
| `enable_detection` | false | Detection subscription |
| `registration` | 1 | 0 / 1; -1 only with depth disabled |
| `doe_power` | 255 | A/AP projector level 0…255; -1 preserves the current value |
| `imu_calib_dir` | empty | XML export directory; empty disables export |
| `publish_tf` | true | Static TF tree |
| `det_frame_id` | empty | Automatic source frame: RGB on A/AP, right on P; explicit overrides supported |

Soft-trigger mode 1 is not supported by this node: it has no trigger service, and `trigger_mode=1` fails startup validation. With `trigger_mode=-1`, preserving a camera already in soft-trigger mode makes each connection attempt fail and the node keeps retrying. When images are enabled, calibration is read only after the first frame. External mode therefore requires an input pulse within `first_frame_wait_ms`; a startup image timeout fails the current connection attempt and the node retries in either automatic or external mode. After initialization, waiting for further external pulses is allowed.

Full stereo + half depth requires post mode; half stereo + full depth is unsupported. Each stream's resolution controls its full output geometry even if ROI is enabled.

For each prefix `stereo_` and `color_`:

| Suffix | Default | Meaning |
| --- | --- | --- |
| `auto_expo` | false | Automatic exposure |
| `exposure_us` | 5000 | Manual exposure, microseconds |
| `gain_x` | 1 | Manual gain multiplier, 1…16 |
| `min_exposure_us / max_exposure_us` | 100 / 65535 | Exposure limits; -1 leaves that limit unchanged |
| `min_gain_x / max_gain_x` | 1 / 16 | Gain limits; -1 leaves that limit unchanged |

Manual values must lie within configured limits. Camera command failures stop initialization. Configuration order is geometry first, then FPS, followed by effective FPS read-back.

Topic overrides: `cam0_topic` (right), `cam1_topic` (left), `color_topic`, `color_info_topic`, `depth_topic`, `depth_info_topic`, `det_topic`, `imu_topic`.
Frame overrides: `base_frame_id`, `cam0_frame_id`, `cam1_frame_id`, `color_frame_id`, `det_frame_id`, `imu_frame_id`. Defaults are listed in the launch file.

## ROI and detection coordinates

ROI parameters follow `roi_<stream>_enable/x1/y1/x2/y2`; streams are `left_raw`, `right_raw`, `color_raw`, `left_rect`, `right_rect`, `color_rect`.

Coordinates are zero-based in that stream's full output image before ROI is applied. Rectified-stream windows use the configured output resolution. Windows are half-open `[x1,x2) × [y1,y2)`, at least 16×16, with even coordinates. Depth cannot be cropped. Only rectified images are published by this node; raw ROI settings still refer to their separate camera streams.

RGB CameraInfo describes the actual cropped image: width/height change, and K/P principal points subtract the ROI origin. Its ROI field stays empty to prevent applying the offset twice. CameraInfo is withheld if the received dimensions disagree with the expected geometry. Detection coordinates are scaled from SEG coordW/coordH, offset by the source ROI, and clipped to the image window. Empty, inverted or non-finite boxes and malformed SEG packets are discarded.

All six ROI settings are sent on each connection. Enabled windows must be accepted by the camera; firmware without ROI support may run only with every ROI disabled. Active windows are cleared during normal shutdown. A known lost connection skips unreachable ROI-reset commands; the requested windows are reapplied after reconnecting.

## Timestamps, connection and shutdown

Default host alignment runs after the first image, or the first IMU sample in IMU-only mode, so camera startup cannot invalidate the offset by resetting its PTS base. Old SDK queues are drained and any in-flight samples from the previous device-time domain are discarded. Zero PTS samples are dropped. IMU camera-time calibration offsets are **not** automatically applied.

`align_time_on_start=false` explicitly retains device PTS for diagnostics; these stamps must not be mixed with host-time sensors or dynamic TF. The built-in `use_sim_time` parameter is checked at startup and must remain `false` throughout live operation.

The ROS node remains running when the camera is absent at startup or a connection is lost. A closed SDK data connection or 3 consecutive failures of the independent UDP online probe stops the current camera session. After cleanup, the node retries every 1 second until the camera becomes available or the user stops the node. SDK connection and device-initialization time are additional to the retry interval.

Each successful connection reapplies settings, waits for initial data, aligns timestamps, rereads calibration and rebuilds CameraInfo/TF before publishing. Every session has fresh queues and buffers, so packets from an old connection are not carried into the new session. The node PID stays the same; publishing endpoints may be recreated and existing subscribers automatically match them again.

Downstream TF buffers may retain previously received static transforms. Replacing a camera or invalidating calibration can require clearing/restarting those downstream buffers; omitting a transform in a later session does not revoke an old cached transform.

The probe checks UDP 5004 every 2 seconds, including when silent packet loss gives TCP no close notification. Normal image timeouts, including waiting for an external trigger while the camera remains online, are allowed. Networks that block UDP 5004 can set connection_probe=false; that disables silent-network-loss detection and retains TCP-close detection. UDP discovery does not detect a fault that filters only TCP while leaving UDP reachable.

Invalid static configuration stops the node at startup. Normal shutdown stops retrying, joins threads and closes the camera; disabling IMU is safe. There is no detached forced-exit watchdog.

## Calibration and XML export

The driver uses SDK 0.5.2 structured calibration getters, with the SDK's legacy-firmware fallback. Camera intrinsics, inverse rectification, baseline, IMU rotations/noise and available intrinsics are validated independently.

```bash
ros2 launch movesense_x95_ros2 movesense_x95.launch.py imu_calib_dir:=/path/to/calibration
```

The export contains `calibration_status.xml` plus:

| Directory | Camera files | IMU reference |
| --- | --- | --- |
| `xml` | M1, D1, M2, D2, IR_1, IR_2, t_P2 | Raw left camera |
| `xml_rgb` | M2, D2, t_P2, iR_2, T_reg, R, T | Raw RGB camera; A/AP only |

File names above have the `.xml` extension. A valid IMU segment adds `r_IMU.xml`, `t_IMU.xml`, `ts_IMU.xml`, `noise_IMU.xml`, `intrinsics_IMU.xml`. A prefers RGB-IMU calibration and AP/P prefer stereo-IMU calibration. When the preferred segment is unavailable, an available valid segment is used instead; the actual stored calibration determines the TF reference.

Exported image matrices retain the SDK's 1280×960 calibration domain; they are not rewritten for output downsampling or ROI. CameraInfo instead describes the configured output image and the RGB crop when enabled.

- `r_IMU.xml` / `t_IMU.xml` describe **IMU → raw camera**, unchanged from SDK data. Translation/baseline use mm; timeshift uses ms, with `t_imu = t_cam + timeshift`. Convert translation from mm to m and timeshift from ms to s where required by downstream software.
- `IR_1/IR_2` are `(K_rect R_rect)^-1`, not rotation matrices. RGB geometry uses `R_C_Lr = iR_2 K_out`, `t_C_Lr = R_C_Lr T_reg`; reserved payload rotation fields are ignored.
- `intrinsics_IMU.xml` contains `model=scale-misalignment`, `has_intrinsics`, and, when available, the four 3×3 matrices `accel_M`, `gyro_M`, `gyro_A`, `gyro_C`. Legacy devices may provide valid extrinsics without these matrices; `has_intrinsics=0` contains no matrix nodes.
- The IMU topic converts raw sensor counts to SI units and preserves the IMU axes; acceleration includes gravity. Reading these matrices does not automatically compensate the samples.
- Static TF composes raw-camera IMU extrinsics with raw→rectified rotation. Exported raw extrinsics and published rectified-camera TF therefore intentionally differ.
- Check `export_complete=1` and validity flags in `calibration_status.xml`. On reuse of an export directory, invalidated files owned by this exporter are overwritten with `valid=0` and old parameter nodes removed; unrelated files are preserved.

Full parameter definitions and compensation formulas: [SDK calibration reference](https://github.com/manplus-ai/MoveSense_X95_SDK/blob/movesense-0.5.2/doc/CALIBRATION_EN.md).

## Common implementation

ROS-independent declarations live under `include/movesense_x95/`. Non-template implementations live in the matching source files: `src/calibration_data.cpp`, `calibration_export.cpp`, `camera_config.cpp`, `camera_session.cpp`, `data_receiver.cpp`, `image_geometry.cpp`, `seg_result.cpp` and `session_supervisor.cpp`. Only template implementations remain in headers. Their contents and the common CMake module are byte-identical in the ROS1 and ROS2 repositories. ROS-specific files adapt logging, parameters, messages and node APIs.

When both repositories are sibling directories, CMake checks the common file list and SHA256 hashes and fails on differences. Either repository can also build independently. `MOVESENSE_VERIFY_PEER_CORE=OFF` disables only this cross-repository check.

Offline regression covers calibration/TF composition, invalid data, depth units, ROI, detection coordinates, SEG bounds, shutdown without initialization and the retry state machine.

AP hardware checks covered 640×480 pre-downsampling, full-resolution stereo with half-resolution RGB/depth in post mode, default RGB+depth at 25 FPS, RGB ROI and disabled IMU. The ROI run received empty detection arrays; nonempty-box coordinate transforms were verified offline. Startup with the camera unavailable, camera-service restart and a 20-second simulated network outage all recovered with the same ROS process PID. The tested camera had valid legacy RGB-IMU extrinsics but no complete IMU intrinsic matrices; the four-matrix path was covered offline. External-trigger pulses, physical cable removal and other camera variants were not part of this acceptance run.

## License

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
