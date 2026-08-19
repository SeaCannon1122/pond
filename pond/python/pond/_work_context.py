import os
import signal
import threading
from typing import Callable


class WorkContext:
    def __init__(self, target: Callable[[threading.Event], None]):

        signal.signal(signal.SIGINT, self.handle_interrupt)
        signal.signal(signal.SIGTERM, self.handle_interrupt)
        self.shutdown_event = threading.Event()
        self._sigint_count = 0

        self._thread = threading.Thread(
            target=target,
            args=(self.shutdown_event,),
            daemon=True,
        )
        self._thread.start()

    def shutdown(self):
        self.shutdown_event.set()

    def handle_interrupt(self, signum, frame):
        self._sigint_count += 1

        print(f"SIGINT {self._sigint_count}/10")

        if self._sigint_count == 1:
            self.shutdown()
        elif self._sigint_count >= 10:
            print("Forced termination.")
            os._exit(1)

    def join(self, timeout=None):
        self._thread.join(timeout)