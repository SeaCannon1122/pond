cmake_minimum_required(VERSION 3.5)

project(quac_modular)

include_directories(pond/include)

add_subdirectory(pond)
add_subdirectory(modules)