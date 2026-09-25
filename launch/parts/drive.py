from pond import Manager

def drive(pm: Manager, real: bool):
    
    pm.set_thread_frame_time("drive_thread", 0.1)

    pm.load_module(
        name="drive_controller",
        bundle_name="controllers",
        module_name="diff_drive_controller",
        thread_name="drive_thread",
        parameters={
            "wheels" : [
                { "joint" : "wheel_front_left_joint",   "radius" : 0.05, "motor_name" : "wheel_front_left_motor"    },
                { "joint" : "wheel_front_right_joint",  "radius" : 0.05, "motor_name" : "wheel_front_right_motor"   },
                { "joint" : "wheel_rear_left_joint",    "radius" : 0.05, "motor_name" : "wheel_back_left_motor"     },
                { "joint" : "wheel_rear_right_joint",   "radius" : 0.05, "motor_name" : "wheel_back_right_motor"    },
            ],
            "slip_multiplier" : 1.6
        },
    )

    if real:
        pm.load_module(
            name="wheel_driver",
            bundle_name="waveshare",
            module_name="ddsm115_driver",
            thread_name="drive_thread",
            parameters={
                "device" : "/dev/quac/wheels",
                "act" : 3,

                "motors" : [
                    { "name" : "wheel_front_left_motor",    "id" : 1,   "invert" : False,   },
                    { "name" : "wheel_front_right_motor",   "id" : 2,   "invert" : True,    },
                    { "name" : "wheel_back_left_motor",     "id" : 3,   "invert" : False,   },
                    { "name" : "wheel_back_right_motor",    "id" : 4,   "invert" : True,    },
                ]
            }
        )
    else:
        pm.load_module(
            name="dummy_wheels",
            bundle_name="utility",
            module_name="dummy_motor",
            thread_name="drive_thread",
            parameters={
                "mode" : "velocity",
                "motor_names": [
                    "wheel_front_left_motor",
                    "wheel_front_right_motor",
                    "wheel_back_left_motor",
                    "wheel_back_right_motor"
                ]
            },
        )

    pm.load_module(
        name="drive_controller_manager",
        bundle_name="utility",
        module_name="motor_controller_manager",
        thread_name="drive_thread",
        parameters={
            "motor_names": [
                "wheel_front_left_motor",
                "wheel_front_right_motor",
                "wheel_back_left_motor",
                "wheel_back_right_motor"
            ]
        },
    )