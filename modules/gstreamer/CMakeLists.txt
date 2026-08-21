project(gstreamer-pond_bundle)

set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)


add_library(gstreamer-pond_bundle SHARED
  rtp_server.cpp
  bundle.cpp
)

target_include_directories(gstreamer-pond_bundle PRIVATE
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(gstreamer-pond_bundle
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

target_link_options(gstreamer-pond_bundle PRIVATE
  -Wl,--no-undefined
)