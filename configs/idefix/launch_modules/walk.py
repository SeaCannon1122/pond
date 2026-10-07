from pond import Manager

class cfg:
    
    thread_name = "walk_thread"

    motor_names = [
        "FL_motor", "FL_leg_motor", "FL_foot_motor",
        "FR_motor", "FR_leg_motor", "FR_foot_motor",
        "BL_motor", "BL_leg_motor", "BL_foot_motor",
        "BR_motor", "BR_leg_motor", "BR_foot_motor"
    ]

    # quadruped controller


def walk(pm: Manager, real: bool = True, fake: bool = False):
        
    pm.set_thread_frame_time(cfg.thread_name, 0.02)

    # walk controller
    pm.load_module(
        "walk_controller", "controllers/quadruped_controller", cfg.thread_name,
    )

    # hardware
    if real:
        pass

    # mock hardware
    if fake:
        pm.load_module(
            "dummy_servos", "utility/dummy_motor", cfg.thread_name,
            
            {"mode" : "position", "motor_names": cfg.motor_names},
        )

    # controller manager
    pm.load_module(
        "quadruped_controller_manager", "utility/motor_controller_manager", cfg.thread_name,

        {"motor_names": cfg.motor_names},
    )