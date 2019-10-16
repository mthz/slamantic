#
#copyright Matthias Schoerghuber (AIT).
#
import argparse
import os

import numpy as np


def inverse_pose(T):
    R = T[0:3, 0:3]
    t = T[0:3, 3]

    oR = R.transpose()
    ot = -oR.dot(t)

    Tinv = np.eye(4)
    Tinv[0:3, 0:3] = oR
    Tinv[0:3, 3] = ot.transpose()

    return Tinv


def split_extrinsic(extrinsic_file):
    print("convert ", extrinsic_file)
    fextrinsic = open(extrinsic_file, 'r')

    header = fextrinsic.readline().split(" ")
    if not header[0] == "frame":
        raise ValueError(f"trajectroy {extrinsic_file} does not start with frame; valid extrisnic file?")

    base_dir = os.path.dirname(extrinsic_file)

    cam0_file = os.path.join(base_dir, "extrinsic_camera0.txt")
    cam1_file = os.path.join(base_dir, "extrinsic_camera1.txt")
    fcam0 = open(cam0_file, 'w')
    fcam1 = open(cam1_file, 'w')

    print(f"writing cam0 {cam0_file}")
    print(f"writing cam1 {cam1_file}")

    fcameras = [fcam0, fcam1]
    T_cw_origin = [None, None]
    T_cw_initialized = [False, False]

    # transform world coordinates into frame coordinates
    # this is for kitti trajectory the first image and is set from the first pose

    for cnt, line in enumerate(fextrinsic):
        line_split = line.split(" ")

        cam_id = int(line_split[1])

        if cam_id >= len(fcameras):
            raise ValueError("script only supports two cameras")

        T_cw = np.matrix(line_split[2:18], dtype=np.float64)
        T_cw.resize((4, 4))

        if not T_cw_initialized[cam_id]:
            T_cw_origin[cam_id] = T_cw
            T_cw_initialized[cam_id] = True

        # transform pose into frame origin
        T_wc = inverse_pose(T_cw)
        T_wo = T_cw_origin[cam_id].dot(T_wc)

        A = np.squeeze(np.asarray(T_wo)).flatten()
        oArray = A[0:12]

        fcameras[cam_id].write(" ".join([str(i) for i in oArray]) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='create kitti evaluation compatible ground truth files from vkitti 2 multi camera extrinsic')

    parser.add_argument("--dataset", help="path to dataset", default=".")

    args = parser.parse_args()

    variations = ["15-deg-right", "15-deg-left", "clone", "fog", "rain", "sunset", "morning", "overcast", "30-deg-left",
                  "30-deg-right"]

    scenes = ["Scene01", "Scene02", "Scene18", "Scene06", "Scene20"]

    for scene in scenes:
        scene_dir = os.path.join(args.dataset, scene)
        if not os.path.exists(scene_dir):
            raise ValueError(f"{scene_dir} does not exist")

        for variation in variations:
            split_extrinsic(os.path.join(scene_dir, variation, "extrinsic.txt"))
