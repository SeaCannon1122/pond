project(pond_manager_py)

find_package(pybind11 REQUIRED)

pybind11_add_module(_pond bindings.cpp)

target_link_libraries(_pond PRIVATE pond_manager)

set_target_properties(_pond PROPERTIES POSITION_INDEPENDENT_CODE ON)

set_target_properties(_pond PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
        "${CMAKE_BINARY_DIR}/python/pond"
)

add_custom_target(copy_python ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/pond"
        "${CMAKE_BINARY_DIR}/python/pond"
)



