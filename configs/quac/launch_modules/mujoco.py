from pond import Manager
import os
from . import drive
from . import arm

def mujoco(pm: Manager):

    pm.set_thread_frame_time("simulation_thread", 0.01)

    pm.load_module(
        "simulator", "mujoco/mujoco", "simulation_thread",
        {
            "robot_path" : os.getenv("POND_QUAC_CONFIG_PATH", "") + "mujoco_robot.xml",
            "actuator_groups" : [
                {
                    "names" : drive.cfg.motor_names,
                    "types" : len(drive.cfg.motor_names)*["velocity"],
                    "feedback_stage_name" : drive.cfg.feedback_stage,
                    "command_stage_name" : drive.cfg.command_stage,
                },
                {
                    "names" : arm.cfg.motor_names,
                    "types" : len(arm.cfg.motor_names)*["position"],
                    "feedback_stage_name" : arm.cfg.feedback_stage,
                    "command_stage_name" : arm.cfg.command_stage,
                },
            ],

            "settings" : 
            {
                "Rendering" : {
                    "OpenGL Effects" : {
                        "Shadow" : False
                    }
                }
            }
        },
    )