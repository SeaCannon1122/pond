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
        name="robot_stater_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="robot_state_thread",
        parameters={"min_time" : 0.10},
        topic_mappings={}
    )

    pm.load_module(
        name="robot_state_tracker",
        bundle_name="robot_state",
        module_name="state_tracker",
        thread_name="robot_state_thread",
        parameters={
            "description_path" : "/home/pilot/pond/config/robot.urdf",
            #"verbose_model_info" : True,
            "description_interval" : 20,
        },
        topic_mappings={}
    )
    

    pm.load_module(
        name="drive_controller_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="drive_thread",
        parameters={"min_time" : 0.1},
        topic_mappings={}
    )

    pm.load_module(
        name="drive_controller",
        bundle_name="controllers",
        module_name="diff_drive_controller",
        thread_name="drive_thread",
        parameters={
            "wheel_joint_names" : [
                "wheel_front_left_joint",
                "wheel_front_right_joint",
                "wheel_rear_left_joint",
                "wheel_rear_right_joint"
            ],
            "wheel_radii" : [
                0.05, 0.05, 0.05, 0.05
            ],
            "slip_multiplier" : 1.6
        },
        topic_mappings={}
    )

    pm.load_module(
        name="wheel_driver",
        bundle_name="waveshare",
        module_name="ddsm115_driver",
        thread_name="drive_thread",
        parameters={
            "device" : "/dev/quac/wheels",
            "act" : 3,
            "motor_count" : 4,

            "motor0.id" : 1,
            "motor0.invert" : False,

            "motor1.id" : 2,
            "motor1.invert" : True,

            "motor2.id" : 3,
            "motor2.invert" : False,

            "motor3.id" : 4,
            "motor3.invert" : True,
        },
        topic_mappings={}
    )

    pm.load_module(
        name="ros2_bridge_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="ros2_bridge_thread",
        parameters={"min_time" : 0.020},
        topic_mappings={}
    )

    pm.load_module(
        name="ros2_bridge",
        bundle_name="ros2",
        module_name="bridge",
        thread_name="ros2_bridge_thread",
        parameters={
            "node_name" : "pond_bridge",
            "topic_count" : 4,

            "topic0.direction" : "POND_TO_ROS",
            "topic0.pond.topic" : "robot_description",
            "topic0.pond.type" : "std::string",
            "topic0.ros.topic" : "robot_description",
            "topic0.ros.type" : "std_msgs::msg::String",
            "topic0.ros.qos.reliable" : True,
            "topic0.ros.qos.depth" : 1,
            "topic0.ros.qos.volatile" : False,

            "topic2.direction" : "POND_TO_ROS",
            "topic2.pond.topic" : "tf_static",
            "topic2.pond.type" : "std::vector<FrameTransform>",
            "topic2.ros.topic" : "tf_static",
            "topic2.ros.type" : "tf2_msgs::msg::TFMessage",
            "topic2.ros.qos.reliable" : True,
            "topic2.ros.qos.depth" : 1,
            "topic2.ros.qos.volatile" : False,

            "topic1.direction" : "POND_TO_ROS",
            "topic1.pond.topic" : "tf",
            "topic1.pond.type" : "std::vector<FrameTransform>",
            "topic1.ros.topic" : "tf",
            "topic1.ros.type" : "tf2_msgs::msg::TFMessage",
            "topic1.ros.qos.reliable" : True,

            "topic3.direction" : "ROS_TO_POND",
            "topic3.pond.topic" : "cmd_vel",
            "topic3.pond.type" : "TwistCommand",
            "topic3.ros.topic" : "/quac/cmd_vel_pilot",
            "topic3.ros.type" : "geometry_msgs::msg::TwistStamped",
            "topic3.ros.qos.reliable" : False,
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