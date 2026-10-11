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

    phase_duration = 0.2

    controller_params = {
        "joint_names" : joint_names,
        "motor_names" : motor_names,

        "timeout" : 0.5,

        "robot_width" : 0.07,
        "robot_length" : 0.26,
        
        "limb_length_hip_to_shoulder" : 0.065,
        "limb_length_thigh" : 0.105,
        "limb_length_shin" : 0.115,

        "T_0" : 0.25 * phase_duration,
        "T_1" : 0.75 * phase_duration,
        "T_st" : phase_duration,
        "T_sw" : phase_duration,

        "step_height" : 0.035,
        "stance_amplitude" : 0.001,

        "max_step_length" : 1.0,
        "stand_height" : 0.181,
        "max_step_speed" : 10.0,
        "jerk" : 18.75 / (phase_duration * phase_duration * phase_duration),
    }


def walk(pm: Manager, real: bool = True, fake: bool = False):
        
    pm.set_thread_frame_time(cfg.thread_name, 0.01)

    # walk controller
    pm.load_module(
        "walk_controller", "controllers/quadruped_controller", cfg.thread_name,
        cfg.controller_params
    )

    # hardware
    if real:
        pass

    # mock hardware
    if fake:
        pm.load_module(
            "dummy_servos", "utility/dummy_motor", cfg.thread_name,
            
            {"mode" : "position", "max_speed" : 10.0, "motor_names": cfg.motor_names},
        )

    # controller manager
    pm.load_module(
        "quadruped_controller_manager", "utility/motor_controller_manager", cfg.thread_name,

        {"frame_time_log" : False, "motor_names": cfg.motor_names},
    )