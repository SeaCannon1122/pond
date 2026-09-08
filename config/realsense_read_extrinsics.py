import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()

# Enable the two stereo IR cameras
config.enable_stream(
    rs.stream.infrared, 1,
    640, 480,
    rs.format.y8, 30
)

config.enable_stream(
    rs.stream.infrared, 2,
    640, 480,
    rs.format.y8, 30
)

profile = pipeline.start(config)

# Get the two camera stream profiles
left = profile.get_stream(
    rs.stream.infrared, 1
).as_video_stream_profile()

right = profile.get_stream(
    rs.stream.infrared, 2
).as_video_stream_profile()

# Get factory-calibrated extrinsics
extrinsics = left.get_extrinsics_to(right)

# Rotation
R = np.array(extrinsics.rotation).reshape(3, 3)

# Translation
T = np.array(extrinsics.translation)

print("Rotation matrix R:")
print(R)

print("\nTranslation T [meters]:")
print(T)

print("\nBaseline [meters]:")
print(np.linalg.norm(T))

pipeline.stop()