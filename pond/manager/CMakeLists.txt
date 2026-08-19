project(pond_manager)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

add_library(pond_manager STATIC
    src/pond_manager.cpp
    src/pond_manager_api.cpp
)

set_target_properties(pond_manager PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)