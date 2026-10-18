project(quadruped_controllers-pond_bundle)

add_library(quadruped_controllers-pond_bundle SHARED
    bundle.cpp
    gait_scheduler.cpp
    trajectory_planner.cpp
    leg_kinematics_controller.cpp
    motor_joint_updater.cpp
)

pond_module_common_setup(quadruped_controllers-pond_bundle)