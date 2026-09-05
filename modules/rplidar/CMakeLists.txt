project(rplidar-pond_bundle)

add_library(rplidar-pond_bundle SHARED
  rplidar.cpp
)

set(RPLIDAR_SDK_DIR ~/lib/rplidar_sdk)

target_include_directories(rplidar-pond_bundle PRIVATE
    ${RPLIDAR_SDK_DIR}/sdk/include
)

target_link_directories(rplidar-pond_bundle PRIVATE
    ${RPLIDAR_SDK_DIR}/output/Linux/Release/
)

target_link_libraries(rplidar-pond_bundle 
    sl_lidar_sdk
)

pond_module_common_setup(rplidar-pond_bundle)