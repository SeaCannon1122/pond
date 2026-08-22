project(orb_slam3-pond_bundle)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM3_DIR ~/lib/ORB_SLAM3)

add_library(orb_slam3-pond_bundle SHARED
  orb_slam3.cpp
)

target_include_directories(orb_slam3-pond_bundle PRIVATE
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
)

target_link_libraries(orb_slam3-pond_bundle
  ORB_SLAM3

  pango_display
  pango_windowing
  pango_core

  OpenGL::GL
  GLEW::GLEW

  pthread
)

target_link_directories(orb_slam3-pond_bundle PRIVATE
  ${ORB_SLAM3_DIR}/lib
)

pond_module_common_setup(orb_slam3-pond_bundle)