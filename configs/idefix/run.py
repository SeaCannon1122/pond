import signal
import time
import subprocess
import os

from pond import Manager

from launch_modules.mujoco import *
from launch_modules.ros2_bridge import *

is_running = True
config_prefix = os.getenv("POND_IDEFIX_CONFIG_PATH", "")

def signal_handler(signum, frame):
    global is_running

    if signum not in (signal.SIGINT, signal.SIGTERM):
        return

    print(" INTERRUPT", flush=True)
    is_running = False

def compile_urdf(destination: str):
    
    with open(destination, "w") as f:
        subprocess.run(
            ["xacro", config_prefix + "urdf/Robodog.urdf.xacro"],
            stdout=f,
            check=True,
        )

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    DESCRIPTION_PATH = config_prefix + "build/robot.urdf"
    compile_urdf(DESCRIPTION_PATH)   

    pm = Manager(True, False)

    pm.set_thread_frame_time("robot_state_thread", 1.0)
    pm.load_module("robot_state_tracker", "robot_state/state_tracker", "robot_state_thread", {"description_path" : DESCRIPTION_PATH})

    #dummy_cam(pm, "camera_front", 1280, 720, 30, True, ip)
    #camera_front(pm, 1280, 720, 30, True, ip)
    #dummy_cam(pm, "camera_back", 1280, 720, 30, True, ip)
    #camera_back(pm, 1280, 720, 30, "color_stereo", True, ip)
    #dummy_cam(pm, "camera_gripper", 1280, 720, 30, True, ip)
    #camera_gripper(pm, 1280, 720, 30, "color_stereo", True, ip)
    # lidar(pm)
    ros2_bridge(pm)
    
    mujoco(pm, DESCRIPTION_PATH)

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()