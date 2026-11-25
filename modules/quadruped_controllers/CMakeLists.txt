project(quadruped_controllers-pond_bundle)

find_package(qpOASES REQUIRED)

add_library(quadruped_controllers-pond_bundle SHARED
    bundle.cpp
    gait_scheduler.cpp
    swing_trajectory_controller.cpp
    leg_kinematics_controller.cpp
    motor_abstractor.cpp
    debug_distributor.cpp
    leg_torque_controller.cpp
    mpc_controller.cpp
    input_synchronizer.cpp
    foot_contact_detector.cpp
    robot_trajectory_planner.cpp
    state_estimator.cpp
    zero_state_estimator.cpp
)

target_link_libraries(quadruped_controllers-pond_bundle qpOASES)

pond_module_common_setup(quadruped_controllers-pond_bundle)