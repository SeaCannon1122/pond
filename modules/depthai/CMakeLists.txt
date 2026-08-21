project(depthai-pond_bundle)

set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(depthai REQUIRED)

add_library(depthai-pond_bundle SHARED
  depthai_camera.cpp
)

target_link_libraries(depthai-pond_bundle
  depthai::core
)

target_link_options(depthai-pond_bundle PRIVATE
  -Wl,--no-undefined
)