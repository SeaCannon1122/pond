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

    pm.set_thread_frame_time("thread", 0.1)

    pm.load_module(
        name="receiver1",
        bundle_name="utility",
        module_name="latency_test_receiver",
        thread_name="thread",
    )

    pm.load_module(
        name="receiver2",
        bundle_name="utility",
        module_name="latency_test_receiver",
        thread_name="thread",
    ) 

    pm.load_module(
        name="receiver3",
        bundle_name="utility",
        module_name="latency_test_receiver",
        thread_name="thread",
    ) 

    pm.load_module(
        name="caller",
        bundle_name="utility",
        module_name="latency_test_caller",
        thread_name="thread",
    )

if __name__ == "__main__":
    main()