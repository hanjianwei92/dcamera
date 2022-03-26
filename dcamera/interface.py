import numpy as np
import time


class DCamera:
    def __init__(self, flip_nums=1):
        self.K = np.eye(3)
        self.depth_scale = 1
        self.flip_nums = flip_nums

    def open(self):
        pass

    def close(self):
        pass

    def get_frame(self):
        return None, None, time.time()
        # rgb image, depth image, timestamp


