import os

import numpy as np
import cv2


def load_depth(depth_path):
    """Load depth image from img_path."""
    # depth_path = depth_path + '_depth.png'
    # print("depth_path", depth_path)
    depth = cv2.imread(depth_path, -1)
    if len(depth.shape) == 3:
        # This is encoded depth image, let's convert
        # NOTE: RGB is actually BGR in opencv
        depth16 = depth[:, :, 1] * 256 + depth[:, :, 2]
        depth16 = np.where(depth16 == 32001, 0, depth16)
        depth16 = depth16.astype(np.uint16)
    elif len(depth.shape) == 2 and depth.dtype == "uint16":
        depth16 = depth
    else:
        assert False, "[ Error ]: Unsupported depth type."
    return depth16


if __name__ == "__main__":
    image = cv2.imread(
        "/home/red0orange/github_projects/CenterSnap/nocs_test_subset/Real/test/scene_1/0006_depth.png"
    )

    image = load_depth("/home/red0orange/data/CenterSnap_test_data/1_depth.png")

    cv2.imshow("image", image)
    cv2.waitKey(0)

    pass
