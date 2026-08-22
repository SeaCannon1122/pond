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

    pm = Manager(False, False)

    pm.load_module(
        name="ros2_bridge",
        bundle_name="ros2",
        module_name="bridge",
        thread_name="ros2_bridge_thread",
        parameters={
            "node_name" : "pond_bridge",
            "topic_count" : 3,

            "topic0.direction" : "POND_TO_ROS",
            "topic0.pond.topic" : "imu_data",
            "topic0.pond.type" : "ImuData",
            "topic0.pond.is_vector" : True,
            "topic0.ros.topic" : "imu",
            "topic0.ros.type" : "sensor_msgs::msg::Imu",
            "topic0.ros.qos.reliable" : False,

            "topic1.direction" : "POND_TO_ROS",
            "topic1.pond.topic" : "camera_transform",
            "topic1.pond.type" : "FrameTransform",
            "topic1.ros.topic" : "pose",
            "topic1.ros.type" : "geometry_msgs::msg::PoseStamped",
            "topic1.ros.qos.reliable" : False,

            "topic2.direction" : "ROS_TO_POND",
            "topic2.pond.topic" : "cmd_vel",
            "topic2.pond.type" : "TwistCommand",
            "topic2.ros.topic" : "cmd_vel",
            "topic2.ros.type" : "geometry_msgs::msg::TwistStamped",
            "topic2.ros.qos.reliable" : False,
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