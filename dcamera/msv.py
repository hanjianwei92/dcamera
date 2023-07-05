# -- coding: utf-8 --
import cv2
import numpy as np
import json
from .msv_camera.MvCameraControl_class import *


class MVSCamera:
    def __init__(self):
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

        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

        init_flag = True
        while init_flag:
            # ch:枚举设备 | en:Enum device, 如果没发现设备就报错
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
            if ret != 0:
                print("enum devices fail! ret[0x%x]" % ret)
                continue

            if deviceList.nDeviceNum == 0:
                print("find no device!")
                continue

            print("Find %d devices!" % deviceList.nDeviceNum)

            # 意义不明的初始化，最好别删除
            for i in range(0, deviceList.nDeviceNum):
                mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
                if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                    print("\ngige device: [%d]" % i)
                    strModeName = ""
                    for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                        if per == 0:
                            break

                        strModeName = strModeName + chr(per)
                    print("device model name: %s" % strModeName)

                    nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                    nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                    nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                    nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                    print("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
                elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                    print("\nu3v device: [%d]" % i)
                    strModeName = ""
                    for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                        if per == 0:
                            break
                        strModeName = strModeName + chr(per)
                    print("device model name: %s" % strModeName)

                    strSerialNumber = ""
                    for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                        if per == 0:
                            break
                        strSerialNumber = strSerialNumber + chr(per)
                    print("user serial number: %s" % strSerialNumber)

            # 如果有多个设备，可以按照序号改变nConnectionNum来控制链接哪个，单个设备直接写0即可
            nConnectionNum = 0
            if int(nConnectionNum) >= deviceList.nDeviceNum:
                print("intput error!")
                continue

            # ch:创建相机实例
            self.cam = MvCamera()

            # ch:选择设备并创建句柄 | en:Select device and create handle
            stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
            ret = self.cam.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                print("create handle fail! ret[0x%x]" % ret)
                continue

            # 打开该设备
            ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                print("open device fail! ret[0x%x]" % ret)
                continue

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

    def get_one_frame(self):
        pData = self.data_buf
        nDataSize = self.nPayloadSize
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        ret = self.cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
        if ret == 0:
            # print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
            #     stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
            temp = np.array(pData)  # 将c_ubyte_Array转化成ndarray得到（3686400，）
            # temp = temp.reshape((5472, 3648, 1))# 根据自己分辨率进行转化
            img = temp.reshape((3648, 5472, 1))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
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
        img = cv2.resize(img, (5472 // 4, 3648 // 4))
        cv2.imshow('ca', img)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            break
