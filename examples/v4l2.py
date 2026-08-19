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

    pm = Manager()

    pm.load_module(
        name="camera",
        bundle_name="quac_modules",
        module_name="v4l2_camera",
        thread_name="default_thread",
        parameters={
            "device": "/dev/video4",
            "width": 640,
            "height": 480,
            "fps": 30
        },
        topic_mappings={
            "out": "image"
        },
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
            "in": "image",
        },
    )

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pm.shutdown_module("camera")
        pm.shutdown_module("streamer")


if __name__ == "__main__":
    main()