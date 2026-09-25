from pond import Manager
import os

def orb_slam3(pm: Manager):
    pm.load_module(
        name="orbslam",
        bundle_name="orb_slam3",
        module_name="slam",
        thread_name="slam_thread",
        parameters={
            "camera_info_path" : os.getenv("POND_CONFIG_PATH", "") + "Realsense.yaml",
            "vocabulary_path" : os.getenv("POND_CONFIG_PATH", "") + "ORBvoc.txt",
            "mode" : "Stereo",
            "frame_id" : "camera",
            "parent_frame_id" : "base_link"
        },
        channel_mappings={
            "stereo_left/image" : "camera_back/stereo_left/image",
            "stereo_right/image" : "camera_back/stereo_right/image",
            "color/image" : "camera_back/color/image",
            "depth/image" : "camera_back/depth/image",
            "imu" : "camera_front/imu"
        },
    )