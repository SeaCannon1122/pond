project(orb_slam3-pond_bundle)

set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM3_DIR ~/lib/ORB_SLAM3)

add_library(orb_slam3-pond_bundle SHARED
  orb_slam3.cpp
)

target_include_directories(orb_slam3-pond_bundle PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${OPENCV_INCLUDE_DIRS}

  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels

  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

target_link_libraries(orb_slam3-pond_bundle
  ORB_SLAM3

  ${OpenCV_LIBS}
  Eigen3::Eigen

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

target_link_options(orb_slam3-pond_bundle PRIVATE
  -Wl,--no-undefined
)