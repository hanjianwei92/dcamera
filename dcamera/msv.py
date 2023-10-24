# -- coding: utf-8 --
import cv2
import numpy as np
import json
from pathlib import Path
import platform

if platform.system() == 'Windows':
    from msv_camera.windows.MvCameraControl_class import *
else:
    from msv_camera.linux.MvCameraControl_class import *


class MVSCamera:
    def __init__(self, ip='192.168.10.44'):
        try:
            with open(str(Path(__file__).parent/'msv_camera/camera_intrinsic.json'), 'r') as f:
                camera_param_dict = json.load(f)
                self.camera_k = np.array(camera_param_dict['camera_intrinsic'])
                self.camera_d = np.array(camera_param_dict['camera_dist'])
        except Exception as e:
            print(e)
            print('请先标定相机')
            self.camera_k = None
            self.camera_d = None

        init_flag = True
        while init_flag:
            # ch:枚举设备
            stDevInfo = MV_CC_DEVICE_INFO()
            stGigEDev = MV_GIGE_DEVICE_INFO()

            deviceIp = ip
            import socket
            netIp = socket.gethostbyname(socket.gethostname())

            deviceIpList = deviceIp.split('.')
            stGigEDev.nCurrentIp = (int(deviceIpList[0]) << 24) | (int(deviceIpList[1]) << 16) | (
                        int(deviceIpList[2]) << 8) | int(deviceIpList[3])

            netIpList = netIp.split('.')
            stGigEDev.nNetExport = (int(netIpList[0]) << 24) | (int(netIpList[1]) << 16) | (
                        int(netIpList[2]) << 8) | int(netIpList[3])

            stDevInfo.nTLayerType = MV_GIGE_DEVICE
            stDevInfo.SpecialInfo.stGigEInfo = stGigEDev

            # ch:创建相机实例
            self.cam = MvCamera()

            # ch:选择设备并创建句柄
            ret = self.cam.MV_CC_CreateHandle(stDevInfo)
            if ret != 0:
                print("create handle fail! ret[0x%x]" % ret)
                sys.exit()

            # 打开该设备
            ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                print("open device fail! ret[0x%x]" % ret)
                continue

            # ch:探测网络最佳包大小(只对GigE相机有效)
            if stDevInfo.nTLayerType == MV_GIGE_DEVICE:
                nPacketSize = self.cam.MV_CC_GetOptimalPacketSize()
                if int(nPacketSize) > 0:
                    ret = self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
                    if ret != 0:
                        print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
                else:
                    print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

            # ch:设置触发模式为off
            ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                print("set trigger mode fail! ret[0x%x]" % ret)
                sys.exit()

            # 获取该设备关于图像的信息，nPayloadSize 和 data_buf 是关键变量
            stParam = MVCC_INTVALUE()
            memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
            ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
            if ret != 0:
                print("get payload size fail! ret[0x%x]" % ret)
                continue
            self.nPayloadSize = stParam.nCurValue
            self.data_buf = (c_ubyte * self.nPayloadSize)()

            # 打开取流的开关
            ret = self.cam.MV_CC_StartGrabbing()
            if ret != 0:
                print("start grabbing fail! ret[0x%x]" % ret)
                continue

            init_flag = False

    def get_one_frame(self, scaled_size=1):
        pData = self.data_buf
        nDataSize = self.nPayloadSize
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        ret = self.cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
        if ret == 0:
            # print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
            #     stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
            temp = np.array(pData)  # 将c_ubyte_Array转化成ndarray得到（3686400，）
            img = temp.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 1))  # 根据自己分辨率进行转化
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (int(img.shape[1] * scaled_size), int(img.shape[0] * scaled_size)))
            return img
        else:
            print("no data[0x%x]" % ret)
            return None

    def close_camera(self):
        # ch:停止取流 | en:Stop grab image
        ret = self.cam.MV_CC_StopGrabbing()
        if ret != 0:
            print("stop grabbing fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:关闭设备 | Close device
        ret = self.cam.MV_CC_CloseDevice()
        if ret != 0:
            print("close deivce fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:销毁句柄 | Destroy handle
        ret = self.cam.MV_CC_DestroyHandle()
        if ret != 0:
            print("destroy handle fail! ret[0x%x]" % ret)
            sys.exit()

        print('successfully closed camera')

    def __del__(self):
        self.close_camera()


if __name__ == "__main__":
    mvs_camera = MVSCamera()
    while True:
        img = mvs_camera.get_one_frame()
        cv2.imshow('ca', img)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            break
