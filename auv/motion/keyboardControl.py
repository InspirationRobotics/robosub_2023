import os
import threading
import time
import rospy
from .robot_control import RobotControl
from ..utils.disarm import disarm

from auv.motion.servo import Servo
servo = Servo()
rospy.init_node("Keyboard", anonymous=True)
rc = RobotControl()
time.sleep(1)


rc.setDepth(0.55)

forward = 0
lateral = 0
yaw = 0
flag = True

def sendData():
    while flag:
        time.sleep(0.05)
        rc.movement(forward=forward, lateral=lateral, yaw=yaw, pitch=0, roll=0)

thread_sensor_updater = threading.Timer(0, sendData)
thread_sensor_updater.daemon = True
thread_sensor_updater.start()

while flag:
    var = input()
    if(var=="a"):
        lateral = -2
    elif(var=="d"):
        lateral = 2
    elif(var=="w"):
        forward = 2
    elif(var=="s"):
        forward = -2
    elif(var=="f"):
        lateral = 0
        forward = 0
        yaw = 0
    elif(var=="k"):
        yaw = -1
    elif(var=="l"):
        yaw = 1
    elif(var=="t"):
       servo.torpedo()
    elif(var=="q"):
        lateral = 0
        forward = 0
        yaw = 0
        disarm()
        flag = False
    else:
        print("Bad Input")

rc.movement(forward=0, lateral=0, yaw=0, pitch=0, roll=0)
rc.movement(forward=0, lateral=0, yaw=0, pitch=0, roll=0)
