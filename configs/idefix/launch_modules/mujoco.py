from pond import Manager
import os
from . import walk

def mujoco(pm: Manager, torque_mode: bool):
    
    pm.set_thread_frame_time("simulation_thread", 0.01)

    pm.load_module(
        "simulator", "mujoco/mujoco", "simulation_thread",
        {
            "robot_path" : os.getenv("POND_IDEFIX_CONFIG_PATH", "") + "mujoco_robot.xml",
            "actuator_groups" : [
                {
                    "names" : walk.cfg.motor_names, 
                    "types" : 12*["torque" if torque_mode else "position"],
                    "feedback_stage_name" : walk.cfg.feedback_stage,
                    "command_stage_name" : walk.cfg.command_stage
                },
            ]
        },
    )