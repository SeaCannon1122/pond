from pond import Manager

def arm(pm: Manager, real: bool):
    pm.load_module(
        name="arm_controller_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="arm_thread",
        parameters={"min_time" : 0.1},
        topic_mappings={},
        topic_namespace=""
    )

    pm.load_module(
        name="arm_controller",
        bundle_name="controllers",
        module_name="tripple_joint_arm_controller",
        thread_name="arm_thread",
        parameters={

        },
        topic_mappings={
            "motor_cmd" : "arm_motor_cmd",
            "motor_feedback" : "arm_motor_feedback"
        },
        topic_namespace=""
    )

    if real:
        pm.load_module(
            name="servo_driver",
            bundle_name="waveshare",
            module_name="servo_driver",
            thread_name="arm_thread",
            parameters={
                "device" : "/dev/quac/servos",
            },
            topic_mappings={
                "motor_cmd" : "arm_motor_cmd",
                "motor_feedback" : "arm_motor_feedback"
            },
            topic_namespace=""
        )
    else:
        pm.load_module(
            name="dummy_arm",
            bundle_name="utility",
            module_name="dummy_motor",
            thread_name="arm_thread",
            parameters={},
            topic_mappings={
                "motor_cmd" : "arm_motor_cmd",
                "motor_feedback" : "arm_motor_feedback"
            },
            topic_namespace=""
        )