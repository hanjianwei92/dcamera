from dcamera.realsense import SelfClbRealsense
import numpy as np
import cv2


if __name__ == "__main__":
    rs = SelfClbRealsense(flip_nums=1)
    # rs.choose_mode("HighResHighAccuracy")
    # rs.choose_mode("HighResHighDensity")
    while True:
        rgb, depth, t = rs.get_frame()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(1000 * depth, alpha=0.3), cv2.COLORMAP_JET)
        images = np.hstack((bgr, depth_colormap))
        cv2.namedWindow('RealSense_depth', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense_depth', images)
        c = cv2.waitKey(1)
        if c == 27:
            rs.close()
            break

