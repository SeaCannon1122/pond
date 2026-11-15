from pond import Manager
import os, shutil
from . import walk

config_prefix = os.getenv("POND_IDEFIX_CONFIG_PATH", "")

def mujoco(pm: Manager, torque_mode: bool):

    ctrl_type = "torque" if torque_mode else "position"
    
    pm.set_thread_frame_time("simulation_thread", 0.001)

    with open(config_prefix + "description/mujoco_robot.xml", "r") as f_src:
        with open(config_prefix + "build/mujoco_robot.xml", "w") as f_dst:
            f_dst.write(f_src.read().replace("</mujoco>", "  <include file=\"mujoco_"+ctrl_type+"_actuators.xml\"/>\n</mujoco>"))
            shutil.copy(config_prefix + "description/mujoco_"+ctrl_type+"_actuators.xml", config_prefix + "build/mujoco_"+ctrl_type+"_actuators.xml")

    pm.load_module(
        "simulator", "mujoco/mujoco", "simulation_thread",
        {
            "robot_path" : config_prefix + "build/mujoco_robot.xml",
            "actuator_groups" : [
                {
                    "names" : walk.cfg.motor_names, 
                    "types" : 12*[ctrl_type],
                    "feedback_stage_name" : walk.cfg.feedback_stage,
                    "command_stage_name" : walk.cfg.command_stage
                },
            ]
        },
    )