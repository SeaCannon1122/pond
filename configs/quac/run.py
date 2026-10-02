import signal
import time
import subprocess
import os

from pond import Manager

from launch_modules.arm import *
from launch_modules.cameras import *
from launch_modules.drive import *
from launch_modules.orb_slam3 import *
from launch_modules.ros2_bridge import *
from launch_modules.sensors import *
from launch_modules.mujoco import *

is_running = True
config_prefix = os.getenv("POND_QUAC_CONFIG_PATH", "")

def signal_handler(signum, frame):
    global is_running

    if signum not in (signal.SIGINT, signal.SIGTERM):
        return

    print(" INTERRUPT", flush=True)
    is_running = False

def compile_urdf(mesh_prefix: str, destination: str):
    
    with open(destination, "w") as f:
        subprocess.run(
            ["xacro", config_prefix + "urdf/robot.urdf.xacro", "mesh_folder:=" + mesh_prefix],
            stdout=f,
            check=True,
        )

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    NATIVE_DESCRIPTION_PATH = config_prefix + "build/robot.urdf"
    ROS_DESCRIPTION_PATH = config_prefix + "build/robot_ros.urdf"

    compile_urdf(config_prefix + "meshes/", NATIVE_DESCRIPTION_PATH)
    compile_urdf("package://quac/meshes/", ROS_DESCRIPTION_PATH)    

    pm = Manager(True, False)

    ip = "192.168.137.26"
    ip = ip

    pm.set_thread_frame_time("robot_state_thread", 1.0)
    pm.load_module("robot_state_tracker", "robot_state/state_tracker", "robot_state_thread", {"description_path" : ROS_DESCRIPTION_PATH})

    #dummy_cam(pm, "camera_front", 1280, 720, 30, True, ip)
    #camera_front(pm, 1280, 720, 30, True, ip)
    #dummy_cam(pm, "camera_back", 1280, 720, 30, True, ip)
    #camera_back(pm, 1280, 720, 30, "color_stereo", True, ip)
    #dummy_cam(pm, "camera_gripper", 1280, 720, 30, True, ip)
    #camera_gripper(pm, 1280, 720, 30, "color_stereo", True, ip)
    # lidar(pm)
    ros2_bridge(pm)
    
    mujoco(pm, NATIVE_DESCRIPTION_PATH)
    
    drive.drive(pm, False, False)
    arm.arm(pm, False, False)

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()