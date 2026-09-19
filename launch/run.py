import signal
import time

from pond import Manager

from parts.arm import *
from parts.cameras import *
from parts.drive import *
from parts.orb_slam3 import *
from parts.ros2_bridge import *
from parts.sensors import *
from parts.state_tracker import *

is_running = True

def signal_handler(signum, frame):
    global is_running

    if signum not in (signal.SIGINT, signal.SIGTERM):
        return

    print(" INTERRUPT", flush=True)
    is_running = False


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    pm = Manager(True, False)

    ip = "192.168.137.26"

    #dummy_cam(pm, "camera_front", 1280, 720, 30, True, ip)
    #camera_front(pm, 1280, 720, 30, True, ip)
    #dummy_cam(pm, "camera_back", 1280, 720, 30, True, ip)
    #camera_back(pm, 1280, 720, 30, "color_stereo", True, ip)
    #dummy_cam(pm, "camera_gripper", 1280, 720, 30, True, ip)
    #camera_gripper(pm, 1280, 720, 30, "color_stereo", True, ip)
    # lidar(pm)
    state_tracker(pm)
    ros2_bridge(pm)
    drive(pm, False)
    arm(pm, False)
    

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()