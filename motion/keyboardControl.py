from robot_control import RobotControl
from servos import servos
import threading
import time
import os

test = servos()
rc = RobotControl()
time.sleep(1)

rc.setDepth(0.35)

forward = 0
lateral = 0
yaw = 0
throttle=0
flag = True

def sendData():
    while flag:
        time.sleep(0.05)
        rc.movement(throttle=throttle, forward=forward, lateral=lateral, yaw=yaw, pitch=0, roll=0)

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
        test.torpedo()
    elif(var=="q"):
        lateral = 0
        forward = 0
        yaw = 0
        os.system("python3 /home/inspiration/auv/devices/disarm.py")
        flag = False
    else:
        print("Bad Input")

rc.movement(throttle=0, forward=0, lateral=0, yaw=0, pitch=0, roll=0)
rc.movement(throttle=0, forward=0, lateral=0, yaw=0, pitch=0, roll=0)
