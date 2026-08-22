project (modules)

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
    )

    target_link_libraries(${TARGET} pond_module_common)

    target_precompile_headers(${TARGET}
        PRIVATE
            <Eigen/Core>
            <Eigen/Geometry>
            <sophus/se3.hpp>
            <sophus/so3.hpp>
            <opencv2/opencv.hpp>
    )

    target_link_options(${TARGET} PRIVATE
        -Wl,--no-undefined
    )

endfunction()

add_subdirectory(camera)
add_subdirectory(depthai)
add_subdirectory(realsense)
add_subdirectory(gstreamer)

add_subdirectory(orb_slam3)
add_subdirectory(rtabmap)

add_subdirectory(waveshare)
add_subdirectory(controllers)

add_subdirectory(ros2)