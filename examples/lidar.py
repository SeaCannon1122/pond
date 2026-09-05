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

    # pm.load_module(
    #     name="robot_stater_frame_timer",
    #     bundle_name="utility",
    #     module_name="frame_timer",
    #     thread_name="default_thread",
    #     parameters={"min_time" : 0.10},
    #     topic_mappings={}
    # )

    pm.load_module(
        name="lidar",
        bundle_name="rplidar",
        module_name="rplidar",
        thread_name="default_thread",
        parameters={
            "channel_type" : "serial",
            "serial_port" : "/dev/quac/lidar",
            "serial_baudrate" : 115200,
            "frame_id" : "laser",
            "angle_compensate" : True
        },
        topic_mappings={}
    )

    pm.load_module(
        name="ros2_bridge",
        bundle_name="ros2",
        module_name="bridge",
        thread_name="default_thread",
        parameters={
            "node_name" : "pond_bridge",
            "topic_count" : 1,

            "topic0.direction" : "POND_TO_ROS",
            "topic0.pond.topic" : "scan",
            "topic0.pond.type" : "LaserScanSPtr",
            "topic0.ros.topic" : "scan",
            "topic0.ros.type" : "sensor_msgs::msg::LaserScan",
        },
        topic_mappings={}
    )

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pm.shutdown_module(name="drive_controller")


if __name__ == "__main__":
    main()