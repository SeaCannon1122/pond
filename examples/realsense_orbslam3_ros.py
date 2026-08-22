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

    disable_depth = True

    pm.load_module(
        name="camera",
        bundle_name="realsense",
        module_name="camera",
        thread_name="default_thread",
        parameters={
            # "serial_number" : "827312072798", # gripper
            "serial_number" : "938422071694", # back
            "disable_emitter" : disable_depth,
            "color.dims": [640, 480],
        },
        topic_mappings={}
    )

    if not disable_depth:
        pm.load_module(
            name="colorizer", 
            bundle_name="camera", 
            module_name="depth_colorizer", 
            thread_name="default_thread",
            parameters={
                "max_depth" : 4000
            },
            topic_mappings={
                "in": "depth/image", 
                "out": "depth_colorized/image"
            }
        )

        pm.load_module(
            name="depth_gst_streamer",
            bundle_name="gstreamer",
            module_name="rtp_server",
            thread_name="default_thread",
            parameters={
                "width": 640,
                "height": 480,
                "port": 5001,
                "ip": "192.168.137.26",
                "format": "BGR8",
            },
            topic_mappings={
                "in": "depth_colorized/image",
            },
        )
    
    pm.load_module(
        name="color_gst_streamer",
        bundle_name="gstreamer",
        module_name="rtp_server",
        thread_name="default_thread",
        parameters={
            "width": 640,
            "height": 480,
            "port": 5000,
            "ip": "192.168.137.26",
            "format": "RGB8",
            "bitrate": 5000,
            "key_int_max:": 60
        },
        topic_mappings={
            "in": "color/image",
        },
    )

    pm.load_module(
        name="mono_left_keypoint_gst_streamer",
        bundle_name="gstreamer",
        module_name="rtp_server",
        thread_name="default_thread",
        parameters={
            "width": 640,
            "height": 480,
            "port": 5002,
            "ip": "192.168.137.26",
            "format": "BGR8",
        },
        topic_mappings={
            "in": "mono_left_with_keypoints/image",
        },
    )

    pm.load_module(
        name="mono_right_gst_streamer",
        bundle_name="gstreamer",
        module_name="rtp_server",
        thread_name="default_thread",
        parameters={
            "width": 640,
            "height": 480,
            "port": 5003,
            "ip": "192.168.137.26",
            "format": "Mono8",
        },
        topic_mappings={
            "in": "mono_right/image",
        },
    )

    pm.load_module(
        name="orbslam",
        bundle_name="orb_slam3",
        module_name="slam",
        thread_name="slam_thread",
        parameters={
            "camera_info_path" : "/home/pilot/pond/config/Realsense.yaml",
            "vocabulary_path" : "/home/pilot/lib/ORB_SLAM3/Vocabulary/ORBvoc.txt",
            "mode" : "Stereo",
            "frame_id" : "camera",
            "parent_frame_id" : "base_link"
        },
        topic_mappings={},
    )

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