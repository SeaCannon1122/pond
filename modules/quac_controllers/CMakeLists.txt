project(quac_controllers-pond_bundle)

add_library(quac_controllers-pond_bundle SHARED
  bundle.cpp
  diff_drive_controller.cpp
  arm_controller.cpp
  arm_controller_old.cpp
  angle_gripper_controller.cpp
)

pond_module_common_setup(quac_controllers-pond_bundle)