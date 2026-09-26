import signal
import time
import subprocess
import os

from pond import Manager

from parts.arm import *
from parts.cameras import *
from parts.drive import *
from parts.orb_slam3 import *
from parts.ros2_bridge import *
from parts.sensors import *
from parts.state_tracker import *
from parts.mujoco import *

is_running = True

def signal_handler(signum, frame):
    global is_running

    if signum not in (signal.SIGINT, signal.SIGTERM):
        return

    print(" INTERRUPT", flush=True)
    is_running = False

def compile_urdf(mesh_prefix: str, destination: str):
    config_prefix = os.getenv("POND_CONFIG_PATH", "")

    with open(destination, "w") as f:
        subprocess.run(
            ["xacro", config_prefix + "urdf/robot.urdf.xacro", "mesh_folder:=" + mesh_prefix],
            stdout=f,
            check=True,
        )

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    config_prefix = os.getenv("POND_CONFIG_PATH", "")

    NATIVE_DESCRIPTION_PATH = config_prefix + "robot.urdf"
    ROS_DESCRIPTION_PATH = config_prefix + "robot_ros.urdf"

    compile_urdf(config_prefix + "meshes/", NATIVE_DESCRIPTION_PATH)
    compile_urdf("package://quac/meshes/", ROS_DESCRIPTION_PATH)    

    pm = Manager(True, False)

    ip = "192.168.137.26"
    ip = ip

    #dummy_cam(pm, "camera_front", 1280, 720, 30, True, ip)
    #camera_front(pm, 1280, 720, 30, True, ip)
    #dummy_cam(pm, "camera_back", 1280, 720, 30, True, ip)
    #camera_back(pm, 1280, 720, 30, "color_stereo", True, ip)
    #dummy_cam(pm, "camera_gripper", 1280, 720, 30, True, ip)
    #camera_gripper(pm, 1280, 720, 30, "color_stereo", True, ip)
    # lidar(pm)
    state_tracker(pm, ROS_DESCRIPTION_PATH)
    ros2_bridge(pm)
    #drive(pm, False)
    #arm(pm, False)

    mujoco(pm, NATIVE_DESCRIPTION_PATH)
    

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()