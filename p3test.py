# import platform
# cams = ["1", "2", "3"]

# for i in enumerate(cams):
#     print(i[0])

# print(platform.node())

# if("nano" in platform.node()):
#     print("true")

# import depthai as dai
# import cv2

# def list_devices(name=None):
#     available_devices = []
#     for device in dai.Device.getAllAvailableDevices():
#         available_devices.append(device.getMxId())
    
#     return available_devices

# devices = list_devices()
# print(devices)

def difference(string1, string2):
    # Split both strings into list items
    string1 = string1.split()
    string2 = string2.split()

    A = set(string1) # Store all string1 list items in set A
    B = set(string2) # Store all string2 list items in set B

    str_diff = A.symmetric_difference(B)
    print(list(str_diff))

difference("/dev/video2 /dev/video3", "/dev/video0 /dev/video2 /dev/video3")