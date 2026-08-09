cmake_minimum_required(VERSION 3.5)

project(quac_modular)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(pond/include)

add_subdirectory(pond)
add_subdirectory(modules)