project(depthai-pond_bundle)

find_package(depthai REQUIRED)

add_library(depthai-pond_bundle SHARED
  depthai_camera.cpp
)

target_link_libraries(depthai-pond_bundle
  depthai::core
)

pond_module_common_setup(depthai-pond_bundle)