from pond import Manager
from .cfg import cfg_drive as cfg

def drive(pm: Manager, real: bool = True, fake: bool = False):
        
    pm.set_thread_frame_time(cfg.thread_name, 0.1)

    # drive controller
    pm.load_module(
        "drive_controller", "controllers/diff_drive_controller", cfg.thread_name,

        {"wheels" : cfg.wheels, "slip_multiplier" : cfg.slip_multiplier},
    )

    # ddsm hardware
    if real:
        pm.load_module(
            "wheel_driver", "waveshare/ddsm115_driver", cfg.thread_name,        
            
            {"device" : cfg.ddsm_device, "act" : cfg.ddsm_act, "motors" : cfg.ddsm_motors}
        )

    # mock hardware
    if fake:
        pm.load_module(
            "dummy_wheels", "utility/dummy_motor", cfg.thread_name,
            
            {"mode" : "velocity", "motor_names": cfg.motor_names},
        )

    # controller manager
    pm.load_module(
        "drive_controller_manager", "utility/motor_controller_manager", cfg.thread_name,

        {"motor_names": cfg.motor_names},
    )