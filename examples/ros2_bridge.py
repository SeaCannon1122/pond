import signal
import time

from pond import Manager

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

    pm.load_module(
        name="ros2_bridge",
        bundle_name="quac_modules",
        module_name="ros2_bridge",
        thread_name="ros2_bridge_thread",
        parameters={
            "pond_topics" : ["imu_data", "pose"],

            "imu_data.pond.type" : "ImuDataStamped.Vector",
            "imu_data.ros.topic" : "imu",
            "imu_data.ros.qos.reliable" : False,

            "pose.pond.type" : "PoseStamped",
            "pose.ros.topic" : "pose",
            "pose.ros.qos.reliable" : False

        },
        topic_mappings={}
    )

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()