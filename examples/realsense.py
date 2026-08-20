import signal
import time

from pond import Manager

is_running = True
int_counter = 0


def signal_handler(signum, frame):
    global is_running, int_counter

    if signum not in (signal.SIGINT, signal.SIGTERM):
        return

    print(" INTERRUPT", flush=True)

    is_running = False

    int_counter += 1

    if int_counter == 10:
        raise SystemExit(187)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    pm = Manager(True, False)

    disable_depth = True

    pm.load_module(
        name="camera",
        bundle_name="quac_modules",
        module_name="realsense_camera",
        thread_name="default_thread",
        parameters={
            "disable_emitter" : disable_depth
        },
        topic_mappings={}
    )

    if not disable_depth:
        pm.load_module(
            name="colorizer", 
            bundle_name="quac_modules", 
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
            bundle_name="quac_modules",
            module_name="gst_server",
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
        bundle_name="quac_modules",
        module_name="gst_server",
        thread_name="default_thread",
        parameters={
            "width": 1280,
            "height": 720,
            "port": 5000,
            "ip": "192.168.137.26",
            "format": "RGB8",
        },
        topic_mappings={
            "in": "color/image",
        },
    )

    pm.load_module(
        name="mono_left_keypoint_gst_streamer",
        bundle_name="quac_modules",
        module_name="gst_server",
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
        bundle_name="quac_modules",
        module_name="gst_server",
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
        bundle_name="quac_modules",
        module_name="orb_slam3",
        thread_name="slam_thread",
        parameters={
            "camera_info_path" : "/home/pilot/pond/config/Realsense.yaml",
            "vocabulary_path" : "/home/pilot/lib/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        },
        topic_mappings={},
    )

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()