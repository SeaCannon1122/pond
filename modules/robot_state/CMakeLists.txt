project(robot_state-pond_bundle)

find_package(urdfdom)

add_library(robot_state-pond_bundle SHARED
  state_tracker.cpp
)

target_link_libraries(robot_state-pond_bundle
  urdfdom::urdfdom_model
)

pond_module_common_setup(robot_state-pond_bundle)