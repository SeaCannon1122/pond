from pond import Manager
from .cfg import cfg_arm as cfg

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