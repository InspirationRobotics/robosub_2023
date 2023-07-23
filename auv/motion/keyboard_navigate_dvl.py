"""This script is used to navigate Onyx with the dvl using the keyboard."""

import os
import time

import rospy

from ..utils import arm, disarm
from .robot_control import RobotControl

rospy.init_node("NavigateDVL", anonymous=True)

rc = RobotControl()
arm.arm()
time.sleep(1)

while not rospy.is_shutdown():
    try:
        inp = input()
        inputs = inp.split(" ")

        if len(inputs) == 3:
            x, y, z = [float(i) for i in inputs]
            h = None
        elif len(inputs) == 4:
            x, y, z, h = [float(i) for i in inputs]
        else:
            print("Invalid input")
            continue

        print(f"[KEYBOARD] X: {x}, Y: {y}, Z: {z}, H: {h}")
        rc.navigate_dvl(x, y, z, h)

    except KeyboardInterrupt:
        break

    except ValueError:
        print("[WARN] Invalid input")
        continue

rc.movement(forward=0, lateral=0, yaw=0, pitch=0, roll=0)
disarm.disarm()
