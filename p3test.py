# import platform
# cams = ["1", "2", "3"]

# for i in enumerate(cams):
#     print(i[0])

# print(platform.node())

# if("nano" in platform.node()):
#     print("true")

import depthai as dai
import cv2

def list_devices(name=None):
    available_devices = []
    for device in dai.Device.getAllAvailableDevices():
        available_devices.append(device.getMxId())
    
    return available_devices

devices = list_devices()
print(devices)