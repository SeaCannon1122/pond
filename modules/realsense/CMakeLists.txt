project(realsense-pond_bundle)

find_package(realsense2 REQUIRED)

add_library(realsense-pond_bundle SHARED
  realsense_camera.cpp
)

target_link_libraries(realsense-pond_bundle
  realsense2
)

pond_module_common_setup(realsense-pond_bundle)