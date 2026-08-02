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

add_library(quac_modules SHARED
  src/bundle.cpp
  src/gst_server.cpp
  src/realsense_driver.cpp
  src/usb_cam_driver.cpp
)

target_include_directories(quac_modules PRIVATE
  ${OPENCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(quac_modules
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  realsense2
)