from pond import Manager

def drive(pm: Manager, real: bool):
    pm.load_module(
        name="drive_controller_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="drive_thread",
        parameters={"min_time" : 0.1},
    )

    pm.load_module(
        name="drive_controller",
        bundle_name="controllers",
        module_name="diff_drive_controller",
        thread_name="drive_thread",
        parameters={
            "wheel_joint_names" : [
                "wheel_front_left_joint",
                "wheel_front_right_joint",
                "wheel_rear_left_joint",
                "wheel_rear_right_joint"
            ],
            "wheel_radii" : [
                0.05, 0.05, 0.05, 0.05
            ],
            "slip_multiplier" : 1.6
        },
        topic_mappings={
            "motor_cmd" : "wheels/motor_cmd",
            "get_motor_feedback" : "wheels/get_motor_feedback"
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
                "motor_count" : 4,

                "motor0.id" : 1,
                "motor0.invert" : False,

                "motor1.id" : 2,
                "motor1.invert" : True,

                "motor2.id" : 3,
                "motor2.invert" : False,

                "motor3.id" : 4,
                "motor3.invert" : True,
            },
            topic_mappings={
                "motor_cmd" : "wheels/motor_cmd",
                "get_motor_feedback" : "wheels/get_motor_feedback"
            },
        )
    else:
        pm.load_module(
            name="dummy_wheels",
            bundle_name="utility",
            module_name="dummy_motor",
            thread_name="drive_thread",
            parameters={"mode" : "velocity"},
            topic_mappings={
                "motor_cmd" : "wheels/motor_cmd",
                "get_motor_feedback" : "wheels/get_motor_feedback"
            },
        )