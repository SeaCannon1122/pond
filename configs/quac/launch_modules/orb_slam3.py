from pond import Manager
import os

def orb_slam3(pm: Manager):
    pm.load_module(
        "orbslam", "orb_slam3/slam", "slam_thread",
        {
            "vocabulary_path" : os.getenv("POND_QUAC_CONFIG_PATH", "") + "ORBvoc.txt",
            "mode" : "Stereo",
        },
        {
            "stereo_left/image" : "camera_back/stereo_left/image",
            "stereo_right/image" : "camera_back/stereo_right/image",
            "color/image" : "camera_back/color/image",
            "depth/image" : "camera_back/depth/image",
        },
    )