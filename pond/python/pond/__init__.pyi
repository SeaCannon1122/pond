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
        topic_mappings: dict[str, str] = {},
        topic_namespace: str = ""
    ) -> str: ...

    def shutdown_module(self, name: str) -> None: ...