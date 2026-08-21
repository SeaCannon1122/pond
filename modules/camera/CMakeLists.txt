project(camera-pond_bundle)

set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

add_library(camera-pond_bundle SHARED
  bundle.cpp
  v4l2_camera.cpp
  dummy_camera.cpp
  depth_colorizer.cpp
)

target_include_directories(camera-pond_bundle PRIVATE
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(camera-pond_bundle
  ${OpenCV_LIBS}
)

target_link_options(camera-pond_bundle PRIVATE
  -Wl,--no-undefined
)