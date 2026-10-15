from pond import Manager
import os

def orb_slam3(pm: Manager, ip: str):
    pm.load_module(
        "orbslam", "orb_slam3/slam", "slam_thread",
        {
            "vocabulary_path" : os.getenv("POND_QUAC_CONFIG_PATH", "") + "ORBvoc.txt",
            "mode" : "RGBD",
            "settings_write_location" : os.getenv("POND_QUAC_CONFIG_PATH", "") + "build/orbslam_config.yaml",
            #"use_imu" : True
        },
        {
            "stereo_left/image" : "camera_back/stereo_left/image",
            "stereo_right/image" : "camera_back/stereo_right/image",
            "stereo_left/info" : "camera_back/stereo_left/info",
            "stereo_right/info" : "camera_back/stereo_right/info",
            "color/image" : "camera_back/color/image",
            "depth/image" : "camera_back/depth_aligned/image",
            "color/info" : "camera_back/color/info",
            "depth/info" : "camera_back/depth_aligned/info",
        },
    )

    pm.load_module(
        "keypoint_image__streamer", "gstreamer/rtp_server", "slam_thread",
        {
            "width": 640, "height": 480,
            "format": "RGB8",

            "ip": ip,
            "port": 5004, 
            
            "bitrate": 5000,
            "key_int_max:": 60
        },
        channel_mappings={"in": "/color_with_keypoints/image",},
    )