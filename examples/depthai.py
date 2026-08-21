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
        name="camera",
        bundle_name="depthai",
        module_name="camera",
        thread_name="default_thread",
        parameters={
            "MxId" : "19443010218B077E00"
        },
        topic_mappings={}
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
            "format": "Mono8",
        },
        topic_mappings={
            "in": "mono_left/image",
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

    

    # pm.load_module(
    #     name="orbslam",
    #     bundle_name="quac_modules",
    #     module_name="orb_slam3",
    #     thread_name="slam_thread",
    #     parameters={
    #         "camera_info_path" : "/home/pilot/pond/config/Realsense.yaml",
    #         "vocabulary_path" : "/home/pilot/lib/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    #     },
    #     topic_mappings={},
    # )

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()