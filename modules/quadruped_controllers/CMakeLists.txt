project(quadruped_controllers-pond_bundle)

find_package(qpOASES REQUIRED)

add_library(quadruped_controllers-pond_bundle SHARED
    bundle.cpp
    gait_scheduler.cpp
    trajectory_planner.cpp
    leg_kinematics_controller.cpp
    motor_joint_updater.cpp
    leg_torque_controller.cpp
    mpc_controller.cpp
)

target_link_libraries(quadruped_controllers-pond_bundle qpOASES)

pond_module_common_setup(quadruped_controllers-pond_bundle)