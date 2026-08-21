project(realsense-pond_bundle)

set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(realsense2 REQUIRED)

add_library(realsense-pond_bundle SHARED
  realsense_camera.cpp
)

target_link_libraries(realsense-pond_bundle
  realsense2
)

target_link_options(realsense-pond_bundle PRIVATE
  -Wl,--no-undefined
)