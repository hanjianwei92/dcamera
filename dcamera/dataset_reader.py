from .interface import DCamera
import numpy as np
import cv2
import os
import time


class DatasetReader(DCamera):
    def __init__(self, dataset_root: str, image_id):
        super(DatasetReader, self).__init__()
        self.dataset_root = dataset_root
        self.image_id = image_id
        self.clk = np.load(os.path.join(self.dataset_root, "clb_k", str(self.image_id) + ".npy"))

    def open(self):
        pass

    def close(self):
        pass

    def get_frame(self):
        color_image = cv2.imread(os.path.join(self.dataset_root, "imgs_rgb", str(self.image_id) + ".jpg"))
        depth_image = np.load(os.path.join(self.dataset_root, "imgs_depth", str(self.image_id) + ".npy"))
        return color_image, depth_image, time.time()