from pond import Manager
import os
from . import walk

def mujoco(pm: Manager):

    config_prefix = os.getenv("POND_IDEFIX_CONFIG_PATH", "")

    xml_path = config_prefix + "mujoco_robot.xml"
    
    pm.set_thread_frame_time("simulation_thread", 0.01)

    pm.load_module(
        "simulator", "mujoco/mujoco", "simulation_thread",
        {
            "robot_path" : xml_path,
            "actuator_groups" : [
                {"names" : walk.cfg.motor_names, "types" : 12*["position"]},
            ]
        },
    )