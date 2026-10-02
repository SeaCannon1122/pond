project(video-pond_bundle)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED 
    libavcodec 
    libavutil 
    libswscale
)

add_library(video-pond_bundle SHARED
  streamer.cpp
  bundle.cpp
)

target_include_directories(video-pond_bundle PRIVATE ${LIBAV_INCLUDE_DIRS})
target_link_libraries(video-pond_bundle ${LIBAV_LIBRARIES} pthread)

pond_module_common_setup(video-pond_bundle)

target_compile_options(video-pond_bundle PRIVATE -O3 -Wall)