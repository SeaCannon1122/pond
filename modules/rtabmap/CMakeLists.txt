project(rtabmap-pond_bundle)

add_library(rtabmap-pond_bundle SHARED
  rtabmap.cpp
)

pond_module_common_setup(rtabmap-pond_bundle)