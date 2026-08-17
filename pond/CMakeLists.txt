project(pond)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

add_library(pond_manager STATIC
    src/manager/pond_manager.cpp
    src/manager/pond_manager_api.cpp
)

add_executable(pond
    src/main.cpp
)

target_link_libraries(pond PRIVATE pond_manager)