from .interface import DCamera
import numpy as np
import cv2
import os
import time


class DatasetReader(DCamera):
    def __init__(self, dataset_root: str, start_image_id=0, cycle_read=True):
        super(DatasetReader, self).__init__()
        if "l515" in dataset_root:
            self.depth_scale = 0.00025
        elif "d435" in dataset_root:
            self.depth_scale = 0.001
        self.dataset_root = dataset_root
        self.image_id = start_image_id
        self.start_image_id = start_image_id
        self.K = np.load(os.path.join(self.dataset_root, "clb_k", str(self.image_id) + ".npy"))
        self.cycle_read = cycle_read

    def open(self):
        pass

    def close(self):
        pass

    def get_frame(self):
        img_path = os.path.join(self.dataset_root, "imgs_rgb", str(self.image_id) + ".jpg")
        depth_path = os.path.join(self.dataset_root, "imgs_depth", str(self.image_id) + ".npy")
        if (not os.path.exists(img_path)) or (not os.path.exists(depth_path)):
            if self.cycle_read:
                self.image_id = self.start_image_id
            else:
                self.image_id -= 1
            img_path = os.path.join(self.dataset_root, "imgs_rgb", str(self.image_id) + ".jpg")
            depth_path = os.path.join(self.dataset_root, "imgs_depth", str(self.image_id) + ".npy")
        color_image = cv2.imread(img_path)
        depth_image = np.load(depth_path)
        self.image_id += 1
        return color_image, depth_image * self.depth_scale, time.time()
