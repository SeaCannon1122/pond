from pond import Manager

def stream_cam(pm: Manager, cam_name: str, width: int, height: int, format: str, ip: str, port: int):
    pm.load_module(
        cam_name+"_streamer", "gstreamer/rtp_server", cam_name+"_thread",
        {
            "width": width, "height": height,
            "format": format,

            "ip": ip,
            "port": port, 
            
            "bitrate": 5000,
            "key_int_max:": 60
        },
        channel_mappings={"in": cam_name+"/color/image",},
    )

# camera front oak d lite
def camera_front(pm: Manager, color_width: int, color_height: int, fps: int, stream: bool = False, ip: str = ""):

    pm.load_module(
        "camera_front", "depthai/camera", "camera_front_thread",
        {
            "camera_name" : "camera_front",
            "MxId" : "19443010218B077E00",
            "fps": fps,

            "color.dims": [color_width, color_height],
            "imu.rate" : 100,
        },
    )

    if stream:
        stream_cam(pm, "camera_front", color_width, color_height, "RGB8", ip, 5000)

# camera back realsense d435
def camera_back(pm: Manager, color_width: int, color_height: int, fps: int, mode: str, stream: bool = False, ip: str = ""):
    pm.load_module(
        "camera_back", "realsense/camera", "camera_back_thread",
        {
            "camera_name" : "camera_back",
            "serial_number" : "827312072798", # gripper
            "fps": fps,
            "mode": mode,

            "color.dims": [color_width, color_height],
            "depth.align_to_color": True,
        },
    )

    if stream:
        stream_cam(pm, "camera_back", color_width, color_height, "RGB8", ip, 5001)

# camera gripper realsense d435
def camera_gripper(pm: Manager, color_width: int, color_height: int, fps: int, mode: str, stream: bool = False, ip: str = ""):
    pm.load_module(
        "camera_gripper", "realsense/camera", "camera_gripper_thread",
        {
            "camera_name" : "camera_gripper",
            "serial_number" : "938422071694", # back
            "fps": fps,
            "mode": mode,

            "color.dims": [color_width, color_height],
            "depth.align_to_color": True,
        },
    )

    if stream:
        stream_cam(pm, "camera_gripper", color_width, color_height, "RGB8", ip, 5002)

# dummy camera
def dummy_cam(pm: Manager, camera_name: str, width: int, height: int, fps: int, stream: bool = False, ip: str = ""):
    pm.load_module(
        camera_name, "utility/dummy_camera", camera_name + "_thread",
        {
            "width": width,
            "height": height,
            "fps": fps,
            "frame_id": camera_name + "_color_optical_frame"
        },
    )

    if stream:
        stream_cam(pm, camera_name, width, height, "RGB8", ip, 5002)