project(ros2-pond_bundle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ros2-pond_bundle SHARED
  ros2_bridge.cpp
)

target_include_directories(ros2-pond_bundle PRIVATE
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(ros2-pond_bundle
  ${OpenCV_LIBS}
  rclcpp::rclcpp
)

ament_target_dependencies(ros2-pond_bundle
  geometry_msgs
  sensor_msgs
)

pond_module_common_setup(ros2-pond_bundle)