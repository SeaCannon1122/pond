from pond import Manager
import subprocess
import os

def state_tracker(pm: Manager):

    config_prefix = os.getenv("POND_CONFIG_PATH", "")

    DESCRIPTION_PATH = config_prefix + "robot.urdf"

    with open(DESCRIPTION_PATH, "w") as f:
        subprocess.run(
            ["xacro", config_prefix + "urdf/robot.urdf.xacro"],
            stdout=f,
            check=True,
        )

    pm.set_thread_frame_time("robot_state_thread", 1.0)

    pm.load_module(
        name="robot_state_tracker",
        bundle_name="robot_state",
        module_name="state_tracker",
        thread_name="robot_state_thread",
        parameters={
            "description_path" : DESCRIPTION_PATH,
            "verbose_model_info": False
        },
    )