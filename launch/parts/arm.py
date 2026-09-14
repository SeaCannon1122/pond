from pond import Manager
import math

def arm(pm: Manager, real: bool):
    pm.load_module(
        name="arm_controller_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="arm_thread",
        parameters={"min_time" : 0.1},
    )

    pm.load_module(
        name="arm_controller",
        bundle_name="controllers",
        module_name="arm_controller",
        thread_name="arm_thread",
        parameters={
            "arm_joint_names": ["arm_segment_0_joint", "arm_segment_1_joint", "arm_segment_2_joint"],
            "gripper_joint_name": "gripper_joint",
            "target_link_name": "arm_end_effector"
        },
        topic_mappings={
            "motor_cmd" : "arm/motor_cmd",
            "get_motor_feedback" : "arm/get_motor_feedback"
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

                "servo0" : {
                    "id" : 1,
                    "invert" : False,
                    "pos_min" : -5*math.pi/4 - 0.1,
                    "pos_max" : 0.0,
                    "offset": 3*math.pi/2
                },
                
                "servo1" : {
                    "id" : 2,
                    "invert" : False,
                    "pos_min" : -math.pi/4,
                    "pos_max" : math.pi/2,
                    "offset": math.pi
                },

                "servo2" : {
                    "id" : 3,
                    "invert" : True,

                    "pos_min" : 0.0,
                    "pos_max" : math.pi - 0.05,
                    "offset": 3*math.pi/2
                },

                # "servo3" : {
                #     "id" : 4,
                #     "invert" : True,
                #     "pos_min" : 0.0,
                #     "pos_max" : 1.7,
                #     "offset": 1.75
                # }
            },
            topic_mappings={
                "motor_cmd" : "arm/motor_cmd",
                "get_motor_feedback" : "arm/get_motor_feedback"
            },
        )
        pass
    else:
        pm.load_module(
            name="dummy_servos",
            bundle_name="utility",
            module_name="dummy_motor",
            thread_name="arm_thread",
            parameters={"mode" : "position"},
            topic_mappings={
                "motor_cmd" : "arm/motor_cmd",
                "get_motor_feedback" : "arm/get_motor_feedback"
            },
        )