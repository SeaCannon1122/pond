from pond import Manager

def lidar(pm: Manager):
    pm.load_module(
        "lidar", "rplidar/lidar", "lidar_thread",
        {
            "channel_type" : "serial",
            "serial_port" : "/dev/quac/lidar",
            "serial_baudrate" : 115200,
            "frame_id" : "laser",
            "angle_compensate" : True
        },
    )