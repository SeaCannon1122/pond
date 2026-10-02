add_executable(pond_debug main.cpp)

target_link_libraries(pond_debug PRIVATE pond_manager)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED 
    libavcodec 
    libavutil 
    libswscale
)

find_package(OpenCV REQUIRED)

add_executable(receiver receiver.cpp)

target_include_directories(receiver PRIVATE ${LIBAV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(receiver PRIVATE ${LIBAV_LIBRARIES} ${OpenCV_LIBRARIES} pthread)

target_compile_options(receiver PRIVATE -O3 -Wall)