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
<<<<<<< HEAD
        return None, None, time.time()
        # rgb image, depth image, timestamp
=======
        return None, None, time.time() # rgb image, depth image, timestamp
>>>>>>> d0d9d11803c9e780d36c487d791786e625b91dc2


