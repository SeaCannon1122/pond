import depthai as dai
import numpy as np

with dai.Device() as device:
    calib = device.readCalibration()

    # Stereo camera sockets — change these if your OAK uses different sockets
    left = dai.CameraBoardSocket.CAM_B
    right = dai.CameraBoardSocket.CAM_C

    # Intrinsics
    K_left = np.array(
        calib.getCameraIntrinsics(left, 1280, 800)
    )

    K_right = np.array(
        calib.getCameraIntrinsics(right, 1280, 800)
    )

    # Distortion
    D_left = np.array(
        calib.getDistortionCoefficients(left)
    )

    D_right = np.array(
        calib.getDistortionCoefficients(right)
    )

    # Extrinsics: left -> right
    T = np.array(
        calib.getCameraTranslationVector(left, right)
    )

    R = np.array(
        calib.getCameraRotationMatrix(left, right)
    )

    print("LEFT INTRINSICS K:")
    print(K_left)

    print("\nRIGHT INTRINSICS K:")
    print(K_right)

    print("\nLEFT DISTORTION:")
    print(D_left)

    print("\nRIGHT DISTORTION:")
    print(D_right)

    print("\nROTATION LEFT -> RIGHT:")
    print(R)

    print("\nTRANSLATION LEFT -> RIGHT:")
    print(T)

    print("\nBASELINE:")
    print(np.linalg.norm(T))