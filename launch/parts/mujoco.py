from pond import Manager

def mujoco(pm: Manager, urdf_path: str):
    pm.set_thread_frame_time("simulation_thread", 0.050)

    pm.load_module(
        name="simulator",
        bundle_name="mujoco",
        module_name="mujoco",
        thread_name="simulation_thread",
        parameters={"urdf_path" : urdf_path},
    )