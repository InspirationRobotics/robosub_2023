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

# def difference(string1, string2):
#    # Split both strings into list items
#    string1 = string1.split()
#    string2 = string2.split()

#    A = set(string1) # Store all string1 list items in set A
#    B = set(string2) # Store all string2 list items in set B

#    str_diff = A.symmetric_difference(B)
#    print(list(str_diff))

# difference("/dev/video2 /dev/video3", "/dev/video0 /dev/video2 /dev/video3")

# from altimu10v5.lsm6ds33 import LSM6DS33
# from altimu10v5.lis3mdl import LIS3MDL
# from time import sleep

# lsm6ds33 = LSM6DS33()
# lis3mdl = LIS3MDL()
# lsm6ds33.enable()
# lis3mdl.enable()

# while True:
#     #print(lsm6ds33.get_accelerometer_g_forces())
#     #print(lsm6ds33.get_gyro_angular_velocity())
#     print(lis3mdl.get_magnetometer_raw())
#     sleep(1)

# import numpy as np
# import math

# A = [10,20,12]
# B = [5,10,13]

# norm1 = np.linalg.norm(A)
# norm2 = math.sqrt(np.dot(A,A))

# print(norm1)
# print(norm2)

# from datetime import datetime

# fileName = str(datetime.now())
# fileName = fileName.split(".")
# fileName = fileName[0].split(" ")
# fileName = fileName[0] + "_" + fileName[1]
# print(fileName)

# import serial
# import os
# import time

# def setPwm(channel, target):
#     target = target*4
#     USB = serial.Serial(port="/dev/ttyACM0")
#     USB.isOpen()
#     lsb = target & 0x7f #7 bits for least significant byte
#     msb = (target >> 7) & 0x7f #shift 7 and take next 7 bits for msb
#     cmd = chr(0x84) + chr(channel) + chr(lsb) + chr(msb)
#     USB.write(bytes(cmd.encode()))
#     USB.close()
# print(createBytes(2, 1500))
# setPwm(2, 1500)

# import json

# SA = ":SA,   +0.00,   +0.00,  0.00"
# TS = ":TS,23070714400162,35.0, +26.1,   0.0,1536.9,  0"
# BI = ":BI,-32768,-32768,-32768,-32768,V"
# BS = ":BS,-32768,-32768,-32768,V"
# BE = ":BE,-32768,-32768,-32768,V"
# BD = ":BD,        +0.00,        +0.00,        +0.00,   0.00,  7.15"

# data = TS

# #print(data.replace(" ", "").split(","))

# def createPacket(SA):
#     dataPacket = {
#         "Timestamp":[],             # year, month, day, hour:minute:second
#         "Attitude":[],              # roll, pitch, and heading in degrees
#         "Salinity":[],              # in ppt (parts per thousand)
#         "Temp":[],                  # celcius
#         "Transducer_depth":[],      # meters
#         "Speed_of_sound":[],        # meters per second
#         "Result_code": [],
#         "DVL_velocity":[],          # mm/s # xyz error
#         "isDVL_velocity_valid": [],   # boolean
#         "AUV_velocity":[],          # mm/s # xyz
#         "isAUV_velocity_valid": [],   # boolean
#         "Distance_from_bottom": [], # meters
#         "Time_since_valid": []      # seconds
#         }
#     SA = SA.replace(" ", "").split(",")
#     if(SA[0]==":SA"):
#         dataPacket["Attitude"] = [float(SA[1]), float(SA[2]), float(SA[3])]
#         #read next line
#         TS = ":TS,23070714400162,35.0, +26.1,   0.0,1536.9,  0"
#         TS = TS.replace(" ", "").split(",")
#         dataPacket["Timestamp"] = ['20'+TS[1][:2]+"-"+TS[1][4:6]+"-"+TS[1][2:4], TS[1][6:8]+":"+TS[1][8:10]+":"+TS[1][10:12]]
#         dataPacket["TimestampSeconds"] = [int(TS[1][6:8])*int(TS[1][8:10])*int(TS[1][10:12])+int(TS[1][12:14])]
#         dataPacket["Salinity"] = float(TS[2])
#         dataPacket["Temp"] = float(TS[3])
#         dataPacket["Transducer_depth"] = float(TS[4])
#         dataPacket["Speed_of_sound"] = float(TS[5])
#         dataPacket["Result_code"] = TS[6]
#         BI = ":BI,-32768,-32768,-32768,-32768,V" # read next line
#         BI = BI.replace(" ", "").split(",")
#         dataPacket["DVL_velocity"] = [int(BI[1]), int(BI[2]), int(BI[3]), int(BI[4])]
#         dataPacket["isDVL_velocity_valid"] = (BI[5]=="A")
#         BS = ":BS,-32768,-32768,-32768,V" #read next line
#         BS = BS.replace(" ", "").split(",")
#         dataPacket["AUV_velocity"] = [int(BS[1]), int(BS[2]), int(BS[3])]
#         dataPacket["isAUV_velocity_valid"] = (BS[4]=="A")
#         BE = ":BE,-32768,-32768,-32768,V" #read next line
#         BE = BE.replace(" ", "").split(",") # unused
#         BD = ":BD,        +0.00,        +0.00,        +0.00,   0.00,  7.15" #read next line
#         BD = BD.replace(" ", "").split(",")
#         dataPacket["Distance_from_bottom"] = float(BD[4])
#         dataPacket["Time_since_valid"] = float(BD[5])
#     return dataPacket

# print(createPacket(SA))

# import json
# import glob
# folderPath = "/home/inspiration/auv/auv/device/cams/models/gateModel/"
# jsonFile = glob.glob(folderPath+"*.json")[0]
# blobFile = glob.glob(folderPath+"*.blob")[0]
# jsonFile = open(jsonFile)

# data = json.load(jsonFile)

# NN_params = data["nn_config"]["NN_specific_metadata"]

# nnBlobPath = blobFile
# classAmt = NN_params["classes"]
# anchors = NN_params["anchors"]
# anchorMasks = NN_params["anchor_masks"]
# labelMap = data["mappings"]["labels"]

# print(labelMap)

# import lsb_release
# if(lsb_release.get_lsb_information()['RELEASE']=="18.04"):
#     print(True)

# from auv.utils import deviceHelper

# usbIDS = [deviceHelper.dataFromConfig("bottomOak"), None]
# ogDev = deviceHelper.findCam(usbIDS)
# print(ogDev)

# import re

# data = ['/dev/video4', '/dev/video6', '/dev/video5', '/dev/video7']

# def num_sort(test_string):
#     return list(map(int, re.findall(r'\d+', test_string)))[0]

# data.sort(key=num_sort)

# print(data)

# params for each detection are: label, confidence, xmin, xmax, ymin, ymax

# import json

# class detection:
#     def __init__(self, label, confidence, xmin, xmax, ymin, ymax):
#         self.label = label
#         self.confidence = confidence
#         self.xmin = xmin
#         self.xmax = xmax
#         self.ymin = ymin
#         self.ymax = ymax

# detections = []

# for i in range(5):
#     detections.append(detection("test"+str(i), i*0.29, i-1, i+1, i-2, i+2))


# test = {}
# for i, val in enumerate(detections):
#     test[str(i)] = [val.label, val.confidence, val.xmin, val.xmax, val.ymin, val.ymax]

# var = str(json.dumps(test))
# var2 = json.loads(var)
# print(var2)

# import Jetson.GPIO as GPIO
# import sys
# import time

# pin1 = 35
# pin2 = 36
# global state
# state = True
# GPIO.setmode(GPIO.BOARD)

# GPIO.setup(pin1, GPIO.OUT)
# GPIO.setup(pin2, GPIO.IN)


# def printState(tp):
#     global state
#     print("Detected")
#     GPIO.cleanup()
#     state = False


# GPIO.output(pin1, GPIO.HIGH)
# print("Its high")

# GPIO.add_event_detect(pin2, GPIO.FALLING, callback=printState, bouncetime=10)

# while state:
#     pass


listTemp = [None, "Hello"]
boardDetect = [100,10]


listTemp = [boardDetect[i] if v is None else v for i,v in enumerate(listTemp)]

print(listTemp)