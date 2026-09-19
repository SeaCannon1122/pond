from pond import Manager

def camera_front(pm: Manager, color_width: int, color_height: int, fps: int, stream: bool = False, ip: str = ""):

    pm.load_module(
        name="camera_front",
        bundle_name="depthai",
        module_name="camera",
        thread_name="camera_front_thread",
        parameters={
            "MxId" : "19443010218B077E00",
            "fps": fps,
            "color": {
                "dims": [color_width, color_height],
                "frame_id" : "camera_front_color_optical_frame",
            },
            "stereo": {
                "dims": [640, 400],
                "left_frame_id" : "camera_front_infra1_optical_frame",
                "right_frame_id" : "camera_front_infra2_optical_frame",
            },
            "imu": {
                "frame_id" : "imu",
                "rate" : 100,
            }  
        },
        channel_namespace="camera_front"
    )

    if stream:
        pm.load_module(
            name="camera_front_streamer",
            bundle_name="gstreamer",
            module_name="rtp_server",
            thread_name="camera_front_thread",
            parameters={
                "width": color_width,
                "height": color_height,
                "port": 5000,
                "ip": ip,
                "format": "RGB8",
                "bitrate": 5000,
                "key_int_max:": 60
            },
            channel_mappings={"in": "camera_front/color/image",},
        )

def camera_back(pm: Manager, color_width: int, color_height: int, fps: int, mode: str, stream: bool = False, ip: str = ""):
    pm.load_module(
        name="camera_back",
        bundle_name="realsense",
        module_name="camera",
        thread_name="camera_back_thread",
        parameters={
            "serial_number" : "827312072798", # gripper
            "fps": fps,
            "color": {
                "dims": [color_width, color_height],
                "frame_id" : "camera_back_color_optical_frame",
            },
            "depth": {
                "dims": [640, 480],
                "frame_id" : "camera_back_depth_optical_frame",
                "align_to_color": True,
            },
            "stereo": {
                "dims": [640, 480],
                "left_frame_id" : "camera_back_infra1_optical_frame",
                "right_frame_id" : "camera_back_infra2_optical_frame",
            },
            "mode": mode
        },
        channel_namespace="camera_back"
    )
    if stream:
        pm.load_module(
            name="camera_back_streamer",
            bundle_name="gstreamer",
            module_name="rtp_server",
            thread_name="camera_back_thread",
            parameters={
                "width": color_width,
                "height": color_height,
                "port": 5001,
                "ip": ip,
                "format": "RGB8",
                "bitrate": 5000,
                "key_int_max:": 60
            },
            channel_mappings={"in": "camera_back/color/image",},
        )

def camera_gripper(pm: Manager, color_width: int, color_height: int, fps: int, mode: str, stream: bool = False, ip: str = ""):
    pm.load_module(
        name="camera_gripper",
        bundle_name="realsense",
        module_name="camera",
        thread_name="camera_gripper_thread",
        parameters={
            "serial_number" : "938422071694", # back
            "fps": fps,
            "color": {
                "dims": [color_width, color_height],
                "frame_id" : "camera_gripper_color_optical_frame",
            },
            "depth": {
                "dims": [640, 480],
                "frame_id" : "camera_gripper_depth_optical_frame",
                "align_to_color": True,
            },
            "stereo": {
                "dims": [640, 480],
                "left_frame_id" : "camera_gripper_infra1_optical_frame",
                "right_frame_id" : "camera_gripper_infra2_optical_frame",
            },
            "mode": mode
        },
        channel_namespace="camera_gripper"
    )

    if stream:
        pm.load_module(
            name="camera_gripper_streamer",
            bundle_name="gstreamer",
            module_name="rtp_server",
            thread_name="camera_gripper_thread",
            parameters={
                "width": color_width,
                "height": color_height,
                "port": 5002,
                "ip": ip,
                "format": "RGB8",
                "bitrate": 5000,
                "key_int_max:": 60
            },
            channel_mappings={"in": "camera_gripper/color/image",},
        )

def dummy_cam(pm: Manager, camera_name: str, width: int, height: int, fps: int, stream: bool = False, ip: str = ""):
    pm.load_module(
        name=camera_name,
        bundle_name="utility",
        module_name="dummy_camera",
        thread_name=camera_name + "_thread",
        parameters={
            "width": width,
            "height": height,
            "fps": fps,
            "frame_id": camera_name + "_color_optical_frame"
        },
        channel_namespace=camera_name
    )

    if stream:
        pm.load_module(
            name=camera_name+"_streamer",
            bundle_name="gstreamer",
            module_name="rtp_server",
            thread_name=camera_name+"_thread",
            parameters={
                "width": width,
                "height": height,
                "port": 5002,
                "ip": ip,
                "format": "RGB8",
                "bitrate": 5000,
                "key_int_max:": 60
            },
            channel_mappings={"in": camera_name+"/color/image",},
        )