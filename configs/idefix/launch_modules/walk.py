from pond import Manager

class cfg:
    
    thread_name = "walk_thread"

    joint_names = [
        "FL", "FL_leg", "FL_foot",
        "FR", "FR_leg", "FR_foot",
        "BL", "BL_leg", "BL_foot",
        "BR", "BR_leg", "BR_foot"
    ]

    motor_names = [joint + "_motor" for joint in joint_names]

    foot_link_names = ["FL_paw", "FR_paw", "BL_paw", "BR_paw"]

    feedback_stage = "read_servo_feedback"
    command_stage = "write_servo_commands"

    trajectory_params = {
        "joint_names" : joint_names,
        "reference_link" : "base_link",

        "foot_base_offset" : [0.015, 0.025, -0.181],

        "step_height" : 0.035,
        "max_step_length" : 1.0,
        "max_step_speed" : 10.0,

        "jerk_times_duration_to_the_3" : 18.75,
        "stance_amplitude" : 0.001,
        "T_0_in_swing_relative" : 0.25,
        "T_1_in_swing_relative" : 0.75,
    }

    kinematics_params = {
        "joint_names" : joint_names,
        "motor_names" : motor_names,
        "foot_link_names" : foot_link_names,   
        "reference_link" : "base_link",
    }


def walk(pm: Manager, real: bool = True, fake: bool = False):
        
    pm.set_thread_frame_time(cfg.thread_name, 0.01)

    # controllers
    pm.load_module("motor_joint_updater", "quadruped_controllers/motor_joint_updater", cfg.thread_name, cfg.kinematics_params)

    pm.load_module("gait_scheduler", "quadruped_controllers/gait_scheduler", cfg.thread_name, {"timeout" : 0.5})

    pm.load_module("trajectory_planner", "quadruped_controllers/trajectory_planner", cfg.thread_name, cfg.trajectory_params)

    pm.load_module("leg_kinematics_controller", "quadruped_controllers/leg_kinematics_controller", cfg.thread_name, cfg.kinematics_params)
        
    # hardware
    if real:
        pass

    # mock hardware
    if fake:
        pm.load_module(
            "dummy_servos", "utility/dummy_motor", cfg.thread_name,
            
            {"mode" : "position", "max_speed" : 10.0, "motor_names": cfg.motor_names, "feedback_stage_name" : cfg.feedback_stage, "command_stage_name" : cfg.command_stage},
        )

    # control flow manager
    pm.load_module(
        "walk_cfm", "utility/control_flow_manager", cfg.thread_name,
        {
            "frame_time_log" : False,

            "stage_names" : [
                "read_servo_feedback",
                "update_joint_states",
                "schedule_gaits",
                "evaluate_trajectories",
                "calculate_joint_angles",
                "write_servo_commands",
            ],
            "data_ids" :
                [motor+"_command" for motor in cfg.motor_names] +
                [motor+"_feedback" for motor in cfg.motor_names] +
                ["gait_phases", "partial_gait_durations", "twist_command", "robot_pose", "foot_target_positions"],

            "data_out" : (len(cfg.motor_names) * 2)*[False] + [True, True, False, False]
        }
    )