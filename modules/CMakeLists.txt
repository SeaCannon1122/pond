project(quac_modules)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
find_package(realsense2 REQUIRED)

add_library(gst_server SHARED
    gst_server/gst_server.cpp
)

target_include_directories(gst_server PRIVATE
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(gst_server
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

add_library(realsense_driver SHARED
    realsense_driver/realsense_driver.cpp
)

target_link_libraries(realsense_driver
  realsense2
)