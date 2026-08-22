project(camera-pond_bundle)

add_library(camera-pond_bundle SHARED
  bundle.cpp
  v4l2_camera.cpp
  dummy_camera.cpp
  depth_colorizer.cpp
)

pond_module_common_setup(camera-pond_bundle)