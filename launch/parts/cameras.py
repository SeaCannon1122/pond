from pond import Manager

def camera_front(pm: Manager):
    pm.load_module(
        name="camera_front",
        bundle_name="depthai",
        module_name="camera",
        thread_name="camera_front_thread",
        parameters={
            "MxId" : "19443010218B077E00",
            "fps": 30,
            "color": {
                "dims": [1280, 720],
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
        topic_mappings={},
        topic_namespace="camera_front"
    )

def camera_front_streamer(pm: Manager, ip: str):
    pm.load_module(
        name="camera_front_streamer",
        bundle_name="gstreamer",
        module_name="rtp_server",
        thread_name="camera_front_thread",
        parameters={
            "width": 1280,
            "height": 720,
            "port": 5000,
            "ip": ip,
            "format": "RGB8",
            "bitrate": 5000,
            "key_int_max:": 60
        },
        topic_mappings={"in": "camera_front/color/image",},
        topic_namespace=""
    )

def camera_back(pm: Manager):
    pm.load_module(
        name="camera_back",
        bundle_name="realsense",
        module_name="camera",
        thread_name="camera_back_thread",
        parameters={
            "serial_number" : "827312072798", # gripper
            "fps": 30,
            "color": {
                "dims": [1280, 720],
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
            "mode": "color_stereo"
        },
        topic_mappings={},
        topic_namespace="camera_back"
    )

def camera_back_streamer(pm: Manager, ip: str):
    pm.load_module(
        name="camera_back_streamer",
        bundle_name="gstreamer",
        module_name="rtp_server",
        thread_name="camera_back_thread",
        parameters={
            "width": 1280,
            "height": 720,
            "port": 5001,
            "ip": ip,
            "format": "RGB8",
            "bitrate": 5000,
            "key_int_max:": 60
        },
        topic_mappings={"in": "camera_back/color/image",},
        topic_namespace=""
    )

def camera_gripper(pm: Manager):
    pm.load_module(
        name="camera_gripper",
        bundle_name="realsense",
        module_name="camera",
        thread_name="camera_gripper_thread",
        parameters={
            "serial_number" : "938422071694", # back
            "fps": 30,
            "color": {
                "dims": [1280, 720],
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
            "mode": "color_stereo"
        },
        topic_mappings={},
        topic_namespace="camera_gripper"
    )

def camera_gripper_streamer(pm: Manager, ip: str):
    pm.load_module(
        name="camera_gripper_streamer",
        bundle_name="gstreamer",
        module_name="rtp_server",
        thread_name="camera_gripper_thread",
        parameters={
            "width": 1280,
            "height": 720,
            "port": 5002,
            "ip": ip,
            "format": "RGB8",
            "bitrate": 5000,
            "key_int_max:": 60
        },
        topic_mappings={"in": "camera_gripper/color/image",},
        topic_namespace=""
    )