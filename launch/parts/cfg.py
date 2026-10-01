import math

class cfg_drive:
    
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

class cfg_arm:
    thread_name = "servo_thread"

    # arm
    arm_joints = ["arm_segment_0_joint", "arm_segment_1_joint", "arm_segment_2_joint"]
    arm_end_effector = "arm_end_effector"
    max_arm_speed = 3.0

    # gripper
    gripper_joint = "gripper_joint"
    gripper_radius = 0.045
    gripper_offset = 0.005

    # hardware
    motor_names = ["arm_motor0", "arm_motor1", "arm_motor2", "gripper_motor"]

    # servos
    servo_device = "/dev/quac/servos"
    servo_baudrate = 1000000

    servo_motors = [
        {
            "name" : motor_names[0],        "offset": 3*math.pi/2,
            "id" : 1,                       "pos_min" : -5*math.pi/4 - 0.1,
            "invert" : False,               "pos_max" : 0.0,
        },
        {
            "name" : motor_names[1],        "offset": math.pi,
            "id" : 2,                       "pos_min" : -math.pi/4,
            "invert" : False,               "pos_max" : math.pi/2,
        },
        {
            "name" : motor_names[2],        "offset": 3*math.pi/2,
            "id" : 3,                       "pos_min" : 0.0,
            "invert" : True,                "pos_max" : math.pi - 0.05,
        },
        {
            "name" : motor_names[3],        "offset"  : 1.75,
            "id" : 4,                       "pos_min" : 0.0,
            "invert" : True,                "pos_max" : 1.7,
        }
    ]