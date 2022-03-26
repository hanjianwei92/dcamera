from .interface import DCamera
import numpy as np
import cv2
import os
import time


class DatasetReader(DCamera):
<<<<<<< HEAD
    def __init__(self, dataset_root: str, image_id):
        super(DatasetReader, self).__init__()
        if "l515" in dataset_root:
            self.depth_scale = 0.00025
        else:
            self.depth_scale = 0.001
        self.dataset_root = dataset_root
        self.image_id = image_id
        self.K = np.load(os.path.join(self.dataset_root, "clb_k", str(self.image_id) + ".npy"))
=======
    def __init__(self, dataset_root: str, start_image_id=0, cycle_read=True, step=1):
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
        self.step = step
>>>>>>> d0d9d11803c9e780d36c487d791786e625b91dc2

    def open(self):
        pass

    def close(self):
        pass

    def get_frame(self):
<<<<<<< HEAD
        color_image = cv2.imread(os.path.join(self.dataset_root, "imgs_rgb", str(self.image_id) + ".jpg"))
        depth_image = np.load(os.path.join(self.dataset_root, "imgs_depth", str(self.image_id) + ".npy"))
        return color_image, depth_image * self.depth_scale, time.time()
=======
        img_path = os.path.join(self.dataset_root, "imgs_rgb", str(self.image_id) + ".jpg")
        depth_path = os.path.join(self.dataset_root, "imgs_depth", str(self.image_id) + ".npy")
        if (not os.path.exists(img_path)) or (not os.path.exists(depth_path)):
            if self.cycle_read:
                self.image_id = self.start_image_id
            else:
                self.image_id -= self.step
            img_path = os.path.join(self.dataset_root, "imgs_rgb", str(self.image_id) + ".jpg")
            depth_path = os.path.join(self.dataset_root, "imgs_depth", str(self.image_id) + ".npy")
        color_image = cv2.imread(img_path)
        depth_image = np.load(depth_path)
        self.image_id += self.step
        return color_image, depth_image * self.depth_scale, time.time()
>>>>>>> d0d9d11803c9e780d36c487d791786e625b91dc2
