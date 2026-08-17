project(quac_modules)

set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(depthai REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM3_DIR ~/lib/ORB_SLAM3)

add_library(quac_modules SHARED
  src/bundle.cpp
  src/gst_server.cpp
  src/realsense_camera.cpp
  src/v4l2_camera.cpp
  src/dummy_camera.cpp
  #src/depthai_driver.cpp
  #src/orbslam3_module.cpp
  #src/ros2_bridge.cpp
)

target_include_directories(quac_modules PRIVATE

  ${OPENCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels

  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

target_link_libraries(quac_modules
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  realsense2

  ORB_SLAM3

  realsense2
  depthai::core
  Eigen3::Eigen

  pango_display
  pango_windowing
  pango_core

  OpenGL::GL
  GLEW::GLEW

  pthread
)

target_link_directories(quac_modules PRIVATE
  ${ORB_SLAM3_DIR}/lib
)

target_link_options(quac_modules PRIVATE
  -Wl,--no-undefined
)