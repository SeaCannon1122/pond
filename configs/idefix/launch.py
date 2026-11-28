import subprocess, os, sys

from pond import Manager
from launch_modules.mujoco import *
from launch_modules.ros2_bridge import *
from launch_modules.walk import *

config_prefix = os.getenv("POND_IDEFIX_CONFIG_PATH", "")

def main():
    build_type = "release" if "-release" in sys.argv else "debug"
    if not ("-docker" in sys.argv):
        os.environ["POND_BUNDLE_PATH"] = f"/home/christopher/pond/build/{build_type}/pond_bundles"

    with open(config_prefix + "build/robot.urdf", "w") as f:
        subprocess.run(["xacro", config_prefix + "description/Robodog.urdf.xacro"], stdout=f, check=True)

    pm = Manager(False, False)

    pm.set_thread_frame_time("robot_state_thread", 1.0)
    pm.load_module("robot_state_tracker", "robot_state/state_tracker", "robot_state_thread", {"description_path" : config_prefix + "build/robot.urdf"})

    mpc_mode = (True if "-mpc" in sys.argv else False)

    mujoco(pm, mpc_mode)

    walk(pm, mpc_mode, False, False)

    ros2_bridge(pm)

    try:
        input("Press Enter to exit...\n")

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()