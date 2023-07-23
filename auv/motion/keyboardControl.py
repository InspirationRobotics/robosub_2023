import os
import threading
import time

import rospy


from ..utils.disarm import disarm
from .robot_control import RobotControl
#from .servo import Servo

#from auv.motion.servo import Servo

#servo = Servo()
rospy.init_node("Keyboard", anonymous=True)

rc = RobotControl()
#servo = Servo()
time.sleep(1)

rc.set_depth(0.55)

forward = 0
lateral = 0
yaw = 0
flag = True


def sendData():
    while flag:
        time.sleep(0.05)
        rc.movement(forward=forward, lateral=lateral, yaw=yaw, pitch=0, roll=0)


thread_mov = threading.Thread(target=sendData)
thread_mov.daemon = True
thread_mov.start()

while flag:
    try:
        var = input()
        if var == "a":
            lateral = -2
        elif var == "d":
            lateral = 2
        elif var == "w":
            forward = 2
        elif var == "s":
            forward = -2
        elif var == "f":
            lateral = 0
            forward = 0
            yaw = 0
        elif var == "k":
            yaw = -1
        elif var == "l":
            yaw = 1
        elif var == "z":
            rc.set_depth(rc.depth - 0.1)
        elif var == "x":
            rc.set_depth(rc.depth + 0.1)
        elif var == "t":
            #servo.torpedo()
            pass
        elif var == "q":
            lateral = 0
            forward = 0
            yaw = 0
            flag = False
            disarm()
            break
        elif var == "m":
            lateral = 0
            forward = 0
            yaw = 0
            flag = False
            break
        else:
            print("Bad Input")
    except KeyboardInterrupt:
        break

flag = False
thread_mov.join()

rc.movement(forward=0, lateral=0, yaw=0, pitch=0, roll=0)
