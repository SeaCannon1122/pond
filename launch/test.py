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
        name="camera_front_info_rate_reducer",
        bundle_name="utility",
        module_name="topic_filter",
        thread_name="ros2_bridge_thread",
        parameters={
            "test" : {"hello", 187},
            "rate": 30,
            "topics_in": {"camera_front/color/cam_info", "a"},
            "topics_out" : {"camera_front/color/cam_info_reduced_rate", "b"}
        },
        topic_mappings={},
        topic_namespace=""
    )    

    try:
        while is_running:
            time.sleep(0.2)

    finally:
        pass


if __name__ == "__main__":
    main()