class Manager:
    def __init__(
        self,
        connect_log: bool,
        distribute_log: bool,
    ) -> None: ...

    def load_module(
        self,
        name: str,
        bundle_name: str,
        module_name: str,
        thread_name: str,
        parameters: dict[str, object],
        topic_mappings: dict[str, str],
    ) -> str: ...

    def shutdown_module(self, name: str) -> None: ...