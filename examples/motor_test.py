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
        name="wheel_driver",
        bundle_name="waveshare",
        module_name="ddsm115_driver",
        thread_name="drive_thread",
        parameters={
            "device" : "/dev/quac/wheels",
            "act" : 3,
            "motor_count" : 1,

            "motor0.id" : 2,
            "motor0.invert" : True,
        },
        topic_mappings={}
    )

    pm.load_module(
        name="motor_tester",
        bundle_name="utility",
        module_name="motor_tester",
        thread_name="drive_thread",
        parameters={
            "velocity" : 1.0,
            "position" : 0.0,
        },
        topic_mappings={}
    )

if __name__ == "__main__":
    main()