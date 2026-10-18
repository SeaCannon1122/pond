project(utility-pond_bundle)

add_library(utility-pond_bundle SHARED
  bundle.cpp
  dummy_camera.cpp
  depth_colorizer.cpp
  dummy_motor.cpp
  channel_filter.cpp
  pond_latency_test.cpp
  control_flow_manager.cpp
)

pond_module_common_setup(utility-pond_bundle)