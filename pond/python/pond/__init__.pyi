class Manager:
    def __init__(
        self,
        connect_log: bool = False,
        distribute_log: bool = False
    ) -> None: ...

    def load_module(
        self,
        name: str,
        bundle_name: str,
        module_name: str,
        thread_name: str = "default_thread",
        parameters: dict[str, object] = {},
        channel_mappings: dict[str, str] = {},
        channel_namespace: str = ""
    ) -> str: ...

    def shutdown_module(self, name: str) -> str: ...

    def print_modules(self) -> str: ...

    def set_thread_frame_time(self, thread_name: str, frame_time: float) -> None: ...