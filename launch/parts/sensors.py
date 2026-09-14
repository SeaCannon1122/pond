from pond import Manager

def lidar(pm: Manager):
    pm.load_module(
        name="lidar",
        bundle_name="rplidar",
        module_name="lidar",
        thread_name="lidar_thread",
        parameters={
            "channel_type" : "serial",
            "serial_port" : "/dev/quac/lidar",
            "serial_baudrate" : 115200,
            "frame_id" : "laser",
            "angle_compensate" : True
        },
    )