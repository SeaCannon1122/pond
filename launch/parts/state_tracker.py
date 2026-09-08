from pond import Manager

def state_tracker(pm: Manager):
    pm.load_module(
        name="robot_stater_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="robot_state_thread",
        parameters={"min_time" : 1.0},
        topic_mappings={},
        topic_namespace=""
    )

    pm.load_module(
        name="robot_state_tracker",
        bundle_name="robot_state",
        module_name="state_tracker",
        thread_name="robot_state_thread",
        parameters={
            "description_path" : "/home/pilot/pond/config/robot.urdf",
        },
        topic_mappings={},
        topic_namespace=""
    )