cmake_minimum_required(VERSION 3.5)

project(bundles)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)

add_library(pond_module_common INTERFACE)

target_link_libraries(pond_module_common
    INTERFACE
        Eigen3::Eigen
        Sophus::Sophus
        ${OpenCV_LIBS}
)

function(pond_module_common_setup TARGET)

    set_target_properties(${TARGET} PROPERTIES
        C_VISIBILITY_PRESET hidden
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES

        C_STANDARD 99
        C_STANDARD_REQUIRED ON

        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF

        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/pond_bundles"
    )

    target_precompile_headers(${TARGET}
        PRIVATE
            <Eigen/Core>
            <Eigen/Geometry>
            <sophus/se3.hpp>
            <sophus/so3.hpp>
            <opencv2/opencv.hpp>
    )

    target_compile_options(${TARGET} PRIVATE
        -Werror=return-type
    )

    target_link_options(${TARGET} PRIVATE
        -Wl,--no-undefined
    )

    target_link_libraries(${TARGET} pond_module_common)

endfunction()

set(ALL_BUNDLES
    depthai
    detection
    gstreamer
    mujoco
    orb_slam3
    quac_controllers
    quadruped_controllers
    realsense
    robot_state
    ros2
    rplidar
    rtabmap
    utility
    v4l2
    video
    waveshare
)

option(BUILD_ALL_BUNDLES "Build all bundles" OFF)

foreach(bundle ${ALL_BUNDLES})
    option(BUILD_BUNDLE_${bundle} "Build bundle in subdirectory ${bundle}" OFF)
    
    if(BUILD_BUNDLE_${bundle} OR BUILD_ALL_BUNDLES)
        add_subdirectory(${bundle})
    endif()
endforeach()