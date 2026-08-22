project(controllers-pond_bundle)

add_library(controllers-pond_bundle SHARED
  bundle.cpp
  diff_drive_controller.cpp
)

pond_module_common_setup(controllers-pond_bundle)