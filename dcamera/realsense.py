import pyrealsense2 as rs
import numpy as np
from .interface import DCamera
import pickle as pkl
import cv2
import time
import json
import os

DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07","0B3A"]
d400_mode_dict = {"HighResHighAccuracy": "HighResHighAccuracyPreset.json",
                  "HighResHighDensity": "HighResHighDensityPreset.json"}
ctx = rs.context()
devs = ctx.query_devices() # devs是device_list类
device_num = devs.size()
devs_dict = {dev.get_info(rs.camera_info.serial_number): dev for dev in devs}


class Realsense(DCamera):
    def __init__(self, fps=30, flip_nums=1, sn=None, l515=False):
        super(Realsense, self).__init__(flip_nums=flip_nums)
        print("exiting devices", devs_dict)
        # Create a pipeline
        self.pipeline = rs.pipeline()
        # Configure depth and color streams
        self.fps = fps
        # Start streaming
        self.config = rs.config()
        if l515 is False:
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
            self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, fps)

        if sn is not None:
            self.config.enable_device(sn)
            self.profile = self.pipeline.start(self.config)
            self.dev = devs_dict[sn]
            self.sn = sn
        else:
            self.profile = self.pipeline.start(self.config)
            self.dev = self.profile.get_device()

        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
        # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, fps)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = self.dev.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", self.depth_scale)

        if l515 :
            depth_sensor.set_option(rs.option.ambient_light, 2)
            ambient_light = depth_sensor.get_option(rs.option.ambient_light)
            print("Ambient light is:", ambient_light)

            min_distance = depth_sensor.set_option(rs.option.min_distance, 190)
            min_distance = depth_sensor.get_option(rs.option.min_distance)
            print("min distance is:", min_distance)

            receiver_gain = depth_sensor.set_option(rs.option.receiver_gain, 18)
            receiver_gain = depth_sensor.get_option(rs.option.receiver_gain)
            print("Receiver Gain is:", receiver_gain)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.K = None
        self.w, self.h = 640, 480

    def get_one_frame(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        timestamp = time.time()
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image * self.depth_scale, timestamp

    def get_frame(self):
        color_images = []
        depth_images = []
        timestamps = []
        for flip in range(self.flip_nums):
            color_image, depth_image, timestamp = self.get_one_frame()
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
        return color_image, depth_image, timestamp

    def open(self):
        self.pipeline.start(self.config)

    def close(self):
        self.pipeline.stop()

    def start(self):
        self.pipeline.start(self.config)

    def stop(self):
        self.pipeline.stop()

    def d400_enable_json(self, json_path):
        dev = self.dev
        advnc_mode = rs.rs400_advanced_mode(dev)
        with open(json_path, "r") as json_f:
            json_string = str(json.load(json_f)).replace("'", '\"')
            advnc_mode.load_json(json_string)
            if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                if dev.supports(rs.camera_info.name):
                    print("This device supports advanced mode:", dev.get_info(rs.camera_info.name))
                else:
                    raise Exception("This device that supports advanced mode was found")
            else:
                raise Exception("This device that supports advanced mode was found")
            print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

            while not advnc_mode.is_enabled():
                print("Trying to enable advanced mode...")
                advnc_mode.toggle_advanced_mode(True)
                # At this point the device will disconnect and re-connect.
                print("Sleeping for 5 seconds...")
                time.sleep(5)
                # The 'dev' object will become invalid and we need to initialize it again
                advnc_mode = rs.rs400_advanced_mode(dev)
                print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

    def choose_mode(self, mode):
        proDir = os.path.split(os.path.realpath(__file__))[0]
        print("Enabling mode", mode)
        path = os.path.join(proDir, "realsense_advance_mode_jsons", d400_mode_dict[mode])
        self.d400_enable_json(path)


class HelpClbRealsense(Realsense):
    def __init__(self, clb_pkl=None, fps=30, flip_nums=1, sn=None):
        if clb_pkl is None:
            import os
            clb_pkl = os.path.join(os.getcwd(), "..", "..", "camera", "realsense2_clb.pkl")

        super(HelpClbRealsense, self).__init__(fps=fps, flip_nums=1, sn=sn)
        with open(clb_pkl, "rb") as pkl_file:
            # Getting intrinsics
            self.intrinsics = pkl.load(pkl_file)
            self.K = self.intrinsics["mtx"]
            self.dist = self.intrinsics["dist"]
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.K, self.dist, None, None, (640, 480), cv2.CV_32FC1)

    def get_one_frame(self):
        color_image, depth_image, timestamp = super(HelpClbRealsense, self).get_frame()
        color_image = cv2.remap(color_image, self.mapx, self.mapy, cv2.INTER_LINEAR)
        depth_image = cv2.remap(depth_image, self.mapx, self.mapy, cv2.INTER_LINEAR)
        return color_image, depth_image, timestamp


class SelfClbRealsense(Realsense):
    def __init__(self, fps=30, flip_nums=1, sn=None, l515=False):
        super(SelfClbRealsense, self).__init__(fps=fps, flip_nums=flip_nums, sn=sn, l515=l515)
        # Getting intrinsics
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.color_intrinsics = color_profile.get_intrinsics()
        self.w, self.h = self.color_intrinsics.width, self.color_intrinsics.height
        K = np.eye(3)
        K[0, 0], K[1, 1], K[0, 2], K[1, 2] = self.color_intrinsics.fx,\
                                             self.color_intrinsics.fy,\
                                             self.color_intrinsics.ppx,\
                                             self.color_intrinsics.ppy
        self.K = K
        self.dist = np.zeros(5)


class SelfClbRealsense_K_D(SelfClbRealsense):
    def __init__(self, fps=30, flip_nums=1, sn=None, l515=False):
        super().__init__(fps=fps, flip_nums=flip_nums, sn=sn, l515=l515)

    def get_frame(self):
        color_images = []
        depth_images = []
        timestamps = []
        for flip in range(self.flip_nums):
            color_image, depth_image, timestamp = super().get_one_frame()
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
        return color_image, depth_image, self.K, self.depth_scale, timestamp

    def get_one_frame(self):
        color_image, depth_image, timestamp = super().get_one_frame()
        return color_image, depth_image, self.K, self.depth_scale, timestamp
