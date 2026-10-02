project(v4l2-pond_bundle)

add_library(v4l2-pond_bundle SHARED
  v4l2_camera.cpp
)

# Required because opencv shared lib lingers around until main exe exits,
# while this module is unloaded before that, but that conflicts with opencv's globals management
target_link_options(v4l2-pond_bundle PRIVATE "-Wl,-z,nodelete")

pond_module_common_setup(v4l2-pond_bundle)