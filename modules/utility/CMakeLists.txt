project(utility-pond_bundle)

add_library(utility-pond_bundle SHARED
  bundle.cpp
  frame_timer.cpp
  dummy_camera.cpp
  depth_colorizer.cpp
  motor_tester.cpp
  dummy_motor.cpp
)

pond_module_common_setup(utility-pond_bundle)