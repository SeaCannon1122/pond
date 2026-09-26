from pond import Manager

def state_tracker(pm: Manager, urdf_path: str):

    pm.set_thread_frame_time("robot_state_thread", 1.0)

    pm.load_module(
        name="robot_state_tracker",
        bundle_name="robot_state",
        module_name="state_tracker",
        thread_name="robot_state_thread",
        parameters={
            "description_path" : urdf_path,
            "verbose_model_info": False
        },
    )