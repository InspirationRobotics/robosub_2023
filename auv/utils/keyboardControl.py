import os
import threading
import time

import rospy


from . import arm, disarm, deviceHelper
from ..motion.robot_control import RobotControl
from ..motion.servo import Servo

rospy.init_node("Keyboard", anonymous=True)


rc = RobotControl()
data = input(f"current depth {rc.depth}, Enter absolute depth\n")
rc.set_depth(float(data))

arm.arm()

sub = deviceHelper.variables.get("sub")
if sub == "onyx":
    servo = Servo()


forward = 0
lateral = 0
yaw = 0
flag = True


def sendData():
    idle = False
    while flag:
        if not idle:
            rc.movement(forward=forward, lateral=lateral, yaw=yaw, pitch=0, roll=0)
            time.sleep(0.05)

        # wanna make sure we do send a stop command before idling
        if forward == 0 and lateral == 0 and yaw == 0:
            idle = True
        else:
            idle = False


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
            rc.set_relative_depth(-0.3)
        elif var == "x":
            rc.set_relative_depth(0.3)
        elif var == "c":
            data = input("Enter absolute depth\n")
            rc.set_depth(float(data))
        elif var == "t" and sub == "onyx":
            servo.torpedo()
            pass
        elif var == "u":
            dist = input("Enter distance to move forward\n")
            rc.forward_dvl(2, float(dist))
        elif var == "b" and sub == "onyx":
            servo.dropper()
            pass
        elif var == "q":
            lateral = 0
            forward = 0
            yaw = 0
            flag = False
            disarm.disarm()
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
        flag = False
        disarm.disarm()
        break

flag = False
thread_mov.join()

rc.movement(forward=0, lateral=0, yaw=0, pitch=0, roll=0)
rospy.signal_shutdown("Keyboard Interrupt")
