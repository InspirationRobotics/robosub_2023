"""
Using the DVL to navigate with Onyx. Takes command line arguments in the form of (x, y, z, h (optional)), where 
x is the number of meters to move laterally, y is the number of meters to move forward, z is the number of meters 
to move either up or down (relative perspective, i.e 3 m up or 3 m down), and h is the target heading (in degrees). 
"""

import os
import time

import rospy

from . import arm, disarm
from ..motion.robot_control import RobotControl

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

        z = 0.5

        print(f"[KEYBOARD] X: {x}, Y: {y}, Z: {z}, H: {h}")
        rc.navigate_dvl(x, y, z, h)

    except KeyboardInterrupt:
        break

    except ValueError:
        print("[WARN] Invalid input")
        continue

rc.movement(forward=0, lateral=0, yaw=0, pitch=0, roll=0)
disarm.disarm()
rospy.signal_shutdown("Rospy Exited")