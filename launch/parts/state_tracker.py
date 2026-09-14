from pond import Manager
import subprocess

def state_tracker(pm: Manager):

    DESCRIPTION_PATH = "/home/pilot/.cache/robot.urdf"

    with open(DESCRIPTION_PATH, "w") as f:
        subprocess.run(
            ["xacro", "/home/pilot/pond/config/urdf/robot.urdf.xacro"],
            stdout=f,
            check=True,
        )

    pm.load_module(
        name="robot_stater_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="robot_state_thread",
        parameters={"min_time" : 1.0},
    )

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