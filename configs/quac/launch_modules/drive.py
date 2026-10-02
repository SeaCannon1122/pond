from pond import Manager

class cfg:
    
    thread_name = "drive_thread"

    motor_names = ["wheel_front_left_motor", "wheel_front_right_motor", "wheel_back_left_motor", "wheel_back_right_motor"]

    # drive controller
    slip_multiplier = 1.6

    wheels = [
        { "joint" : "wheel_front_left_joint",   "radius" : 0.05, "motor_name" : motor_names[0]},
        { "joint" : "wheel_front_right_joint",  "radius" : 0.05, "motor_name" : motor_names[1]},
        { "joint" : "wheel_back_left_joint",    "radius" : 0.05, "motor_name" : motor_names[2]},
        { "joint" : "wheel_back_right_joint",   "radius" : 0.05, "motor_name" : motor_names[3]},
    ]

    # hardware
    ddsm_device = "/dev/quac/wheels"
    ddsm_act = 3

    ddsm_motors = [
        { "name" : motor_names[0],  "id" : 1,   "invert" : False,   },
        { "name" : motor_names[1],  "id" : 2,   "invert" : True,    },
        { "name" : motor_names[2],  "id" : 3,   "invert" : False,   },
        { "name" : motor_names[3],  "id" : 4,   "invert" : True,    },
    ]

def drive(pm: Manager, real: bool = True, fake: bool = False):
        
    pm.set_thread_frame_time(cfg.thread_name, 0.1)

    # drive controller
    pm.load_module(
        "drive_controller", "controllers/diff_drive_controller", cfg.thread_name,

        {"wheels" : cfg.wheels, "slip_multiplier" : cfg.slip_multiplier},
    )

    # ddsm hardware
    if real:
        pm.load_module(
            "wheel_driver", "waveshare/ddsm115_driver", cfg.thread_name,        
            
            {"device" : cfg.ddsm_device, "act" : cfg.ddsm_act, "motors" : cfg.ddsm_motors}
        )

    # mock hardware
    if fake:
        pm.load_module(
            "dummy_wheels", "utility/dummy_motor", cfg.thread_name,
            
            {"mode" : "velocity", "motor_names": cfg.motor_names},
        )

    # controller manager
    pm.load_module(
        "drive_controller_manager", "utility/motor_controller_manager", cfg.thread_name,

        {"motor_names": cfg.motor_names},
    )