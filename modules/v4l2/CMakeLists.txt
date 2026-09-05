project(v4l2-pond_bundle)

add_library(v4l2-pond_bundle SHARED
  v4l2_camera.cpp
)

pond_module_common_setup(v4l2-pond_bundle)