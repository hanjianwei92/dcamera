from dcamera.mechmind_config.CameraClient import CameraClient, ImageType, Command, CameraIntri
import sys
import cv2
import numpy as np
import os
import open3d
from .interface import DCamera
import time


class Mechmind(DCamera):
    def __init__(self, flip_nums=1, camera_ip="192.168.5.101"):
        super(Mechmind, self).__init__(flip_nums=flip_nums)
        self.camera = CameraClient()
        if not self.camera.connect(camera_ip):
            print('Connect falied!')
        fx, fy, u, v = self.camera.getCameraIntri()
        self.K = np.array([[fx, 0, u],
                           [0, fy, v],
                           [0, 0, 1]])
        self.depth_scale = 1

    def get_one_frame(self):
        depth = self.camera.captureDepthImg()
        color = self.camera.captureColorImg()
        timestamp = time.time()
        return color, depth * self.depth_scale, self.K, self.depth_scale, timestamp

    def get_frame(self):
        color_images = []
        depth_images = []
        timestamps = []
        for flip in range(self.flip_nums):
            color_image, depth_image, k_, depth_scale_, timestamp = self.get_one_frame()
            color_images.append(color_image)
            depth_images.append(depth_image)
            timestamps.append(timestamp)
        color_image = np.mean(np.array(color_images), axis=0).astype(np.uint8)
        depth_image = np.mean(np.array(depth_images), axis=0)
        timestamp = np.mean(np.array(timestamps), axis=0)
        flag = np.zeros_like(depth_image).astype(np.bool)
        for d_img in depth_images:
            flag = flag | (d_img < self.depth_scale)
        depth_image[flag] = 0
        depth_image /= 1000.0
        return color_image[:, :, ::-1], depth_image, self.K, self.depth_scale, timestamp
