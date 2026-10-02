from pond import Manager
import math

class cfg:
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

def arm(pm: Manager, real: bool = True, fake: bool = False):

    pm.set_thread_frame_time(cfg.thread_name, 0.05)

    # arm controller
    # pm.load_module(
    #     "arm_controller", "controllers/arm_controller", cfg.thread_name,
        
    #     {"arm_joint_names": cfg.arm_joints, "target_link_name": cfg.arm_end_effector, "max_angle_error": 0.3},
    # )

    pm.load_module(
        "arm_controller", "controllers/arm_controller_old", cfg.thread_name,
        
        {"arm_joint_names": cfg.arm_joints, "target_link_name": cfg.arm_end_effector},
    )

    # gripper controller
    pm.load_module(
        "gripper_controller", "controllers/angle_gripper_controller", cfg.thread_name,
        
        {"joint_name": cfg.gripper_joint, "radius": cfg.gripper_radius, "offset": cfg.gripper_offset},
    )

    # servo hardware
    if real:
        pm.load_module(
            "servo_driver", "waveshare/servo_driver", cfg.thread_name,
            
            {"device" : cfg.servo_device, "baudrate" : cfg.servo_baudrate, "servos" : cfg.servo_motors},
        )

    # mock hardware
    if fake:
        pm.load_module(
            "dummy_servos", "utility/dummy_motor", cfg.thread_name,
            
            {"mode" : "position", "max_speed" : cfg.max_arm_speed, "motor_names": cfg.motor_names},
        )

    # controller manager
    pm.load_module(
        "arm_controller_manager", "utility/motor_controller_manager", cfg.thread_name,
    
        {"motor_names": cfg.motor_names},
    )