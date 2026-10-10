cmake_minimum_required(VERSION 3.5)

project(pond_system)

include_directories(pond/include)

add_subdirectory(pond)
add_subdirectory(modules)
add_subdirectory(debug)