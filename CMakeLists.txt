cmake_minimum_required(VERSION 3.10)
project(movesense_x95_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(MoveSense_X95_SDK REQUIRED)

add_executable(${PROJECT_NAME}
  src/movesense_x95_ros2.cpp
  src/camera_driver.cpp
  src/stereo_calib.cpp
  src/stereo_receiver.cpp
  src/imu_receiver.cpp
  src/stereo_publisher.cpp
  src/imu_publisher.cpp
  src/tf_publisher.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}
  MoveSense::X95_SDK
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
