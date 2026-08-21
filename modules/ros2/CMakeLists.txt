project(ros2-pond_bundle)

set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ros2-pond_bundle SHARED
  ros2_bridge
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

target_link_options(ros2-pond_bundle PRIVATE
  -Wl,--no-undefined
)