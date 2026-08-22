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
        name="drive_controller",
        bundle_name="controllers",
        module_name="diff_drive_controller",
        thread_name="drive_thread",
        parameters={
            "rate" : 10,
            "wheel_joint_names" : [
                "wheel_front_left_joint",
                "wheel_front_right_joint",
                "wheel_back_left_joint",
                "wheel_back_right_joint"
            ],
            "wheel0.pos" : [],
        },
        topic_mappings={}
    )

    pm.load_module(
        name="wheel_driver",
        bundle_name="waveshare",
        module_name="ddsm115_driver",
        thread_name="drive_thread",
        parameters={
            "act" : 3,
            "motor_count" : 4,

            "motor0.id" : 1,
            "motor0.invert" : False,

            "motor1.id" : 2,
            "motor1.invert" : False,

            "motor2.id" : 3,
            "motor2.invert" : False,

            "motor3.id" : 4,
            "motor3.invert" : False,
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