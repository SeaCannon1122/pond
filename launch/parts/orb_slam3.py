from pond import Manager

def orb_slam3(pm: Manager):
    pm.load_module(
        name="orbslam",
        bundle_name="orb_slam3",
        module_name="slam",
        thread_name="slam_thread",
        parameters={
            "camera_info_path" : "/home/pilot/pond/config/Realsense.yaml",
            "vocabulary_path" : "/home/pilot/lib/ORB_SLAM3/Vocabulary/ORBvoc.txt",
            "mode" : "Stereo",
            "frame_id" : "camera",
            "parent_frame_id" : "base_link"
        },
        topic_mappings={
            "stereo_left/image" : "camera_back/stereo_left/image",
            "stereo_right/image" : "camera_back/stereo_right/image",
            "color/image" : "camera_back/color/image",
            "depth/image" : "camera_back/depth/image",
            "imu" : "camera_front/imu"
        },
    )