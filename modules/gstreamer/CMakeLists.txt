project(gstreamer-pond_bundle)

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

pond_module_common_setup(gstreamer-pond_bundle)