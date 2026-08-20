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
        name="wrong_camera",
        bundle_name="quac",
        module_name="realsense_camera",
        thread_name="default_thread",
        parameters={},
        topic_mappings={}
    )

    pm.load_module(
            name="wrong_camera",
            bundle_name="quac_modules",
            module_name="realsense",
            thread_name="default_thread",
            parameters={},
            topic_mappings={}
        )

    pm.load_module(
        name="camera",
        bundle_name="quac_modules",
        module_name="realsense_camera",
        thread_name="default_thread",
        parameters={},
        topic_mappings={}
    )

    pm.load_module(
        name="streamer",
        bundle_name="quac_modules",
        module_name="gst_server",
        thread_name="default_thread",
        parameters={
            "width": 640,
            "height": 480,
            "port": 5000,
            "ip": "192.168.137.26",
            "format": "BGR8",
        },
        topic_mappings={
            "in": "color/cam_info",
        },
    )

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()