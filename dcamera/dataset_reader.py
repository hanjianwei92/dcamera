from .interface import DCamera
import numpy as np
import cv2
import os
import time


class DatasetReader(DCamera):
    def __init__(self, dataset_root: str, start_image_id):
        super(DatasetReader, self).__init__()
        if "l515" in dataset_root:
            self.depth_scale = 0.00025
        elif "d435" in dataset_root:
            self.depth_scale = 0.001
        self.dataset_root = dataset_root
        self.image_id = start_image_id
        self.K = np.load(os.path.join(self.dataset_root, "clb_k", str(self.image_id) + ".npy"))

    def open(self):
        pass

    def close(self):
        pass

    def get_frame(self):
        color_image = cv2.imread(os.path.join(self.dataset_root, "imgs_rgb", str(self.image_id) + ".jpg"))
        depth_image = np.load(os.path.join(self.dataset_root, "imgs_depth", str(self.image_id) + ".npy"))
        self.image_id += 1
        return color_image, depth_image * self.depth_scale, time.time()
