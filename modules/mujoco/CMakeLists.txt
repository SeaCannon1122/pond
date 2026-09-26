project(mujoco-pond_bundle)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

add_library(mujoco-pond_bundle SHARED mujoco.cpp)

target_link_libraries(mujoco-pond_bundle mujoco::mujoco mujoco::libmujoco_simulate glfw)

target_compile_options(mujoco-pond_bundle PRIVATE
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Debug>:-gdwarf-4>
)

pond_module_common_setup(mujoco-pond_bundle)