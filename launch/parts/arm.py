from pond import Manager
import math

def arm(pm: Manager, real: bool):

    pm.set_thread_frame_time("arm_thread", 0.1)

    pm.load_module(
        name="arm_controller",
        bundle_name="controllers",
        module_name="arm_controller",
        thread_name="arm_thread",
        parameters={
            "arm_joint_names": ["arm_segment_0_joint", "arm_segment_1_joint", "arm_segment_2_joint"],
            "target_link_name": "arm_end_effector",
            "max_angle_error": 0.3
        },
    )

    pm.load_module(
        name="gripper_controller",
        bundle_name="controllers",
        module_name="angle_gripper_controller",
        thread_name="arm_thread",
        parameters={
            "joint_name": "gripper_joint",
            "radius": 0.045,
            "offset": 0.005
        },
    )

    if real:
        pm.load_module(
            name="servo_driver",
            bundle_name="waveshare",
            module_name="servo_driver",
            thread_name="arm_thread",
            parameters={
                "device" : "/dev/quac/servos",
                "baudrate" : 1000000,

                "servos" : [
                    {
                        "name" : "arm_motor0",
                        "id" : 1,
                        "invert" : False,
                        "pos_min" : -5*math.pi/4 - 0.1,
                        "pos_max" : 0.0,
                        "offset": 3*math.pi/2
                    },
                    {
                        "name" : "arm_motor1",
                        "id" : 2,
                        "invert" : False,
                        "pos_min" : -math.pi/4,
                        "pos_max" : math.pi/2,
                        "offset": math.pi
                    },
                    {
                        "name" : "arm_motor2",
                        "id" : 3,
                        "invert" : True,
                        "pos_min" : 0.0,
                        "pos_max" : math.pi - 0.05,
                        "offset": 3*math.pi/2
                    },
                    # {
                    #     "name" : "gripper_motor",
                    #     "id" : 4,
                    #     "invert" : True,
                    #     "pos_min" : 0.0,
                    #     "pos_max" : 1.7,
                    #     "offset": 1.75
                    # }
                ]

            },
        )
        pass
    else:
        pm.load_module(
            name="dummy_wheels",
            bundle_name="utility",
            module_name="dummy_motor",
            thread_name="arm_thread",
            parameters={
                "mode" : "position",
                "motor_names": ["arm_motor0", "arm_motor1", "arm_motor2", "gripper_motor"]
            },
        )

    pm.load_module(
        name="arm_controller_manager",
        bundle_name="utility",
        module_name="motor_controller_manager",
        thread_name="arm_thread",
        parameters={"motor_names": ["arm_motor0", "arm_motor1", "arm_motor2", "gripper_motor"]},
    )