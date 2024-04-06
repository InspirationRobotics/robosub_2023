"""
To allow manual input from the keyboard to move the sub, and set specific keys to do specific things
"""

# For interacting with the operating system, being able to create threads, time-related functionalities, respectively
import os
import threading
import time

import rospy

# Local modules
from . import arm, disarm, deviceHelper # For arming, disarming the sub as well as getting the configuration of a device plugged into the sub
from ..motion.robot_control import RobotControl # For motion control (PWM values)
from ..motion.servo import Servo # For controlling the dropper, gripper, and torpedoes (PWM values)

# Initialize ROS node, RobotControl instance
rospy.init_node("Keyboard", anonymous=True)
rc = RobotControl()

# Set depth of the sub using manual input
data = input(f"current depth {rc.depth}, Enter absolute depth\n")
rc.set_depth(float(data))

# Arm the sub (give it autonomous capabilities)
arm.arm()

# If the sub is Onyx, initialize a servo instance (Graey does not have servos)
sub = deviceHelper.variables.get("sub")
if sub == "onyx":
    servo = Servo()

# Set initial values for movement
forward = 0
lateral = 0
yaw = 0
flag = True

def sendData():
    """To continously send movement commands to the sub"""
    idle = False
    while flag:
        if not idle:
            rc.movement(forward=forward, lateral=lateral, yaw=yaw, pitch=0, roll=0)
            time.sleep(0.05)

        # Make sure we send a stop command before idling
        if forward == 0 and lateral == 0 and yaw == 0:
            idle = True
        else:
            idle = False

# Create and start a thread for sending movement data
thread_mov = threading.Thread(target=sendData)
thread_mov.daemon = True
thread_mov.start()

"""
Input to movement for the while loop

'a' : lateral - 2
'd' : lateral + 2
'w' : forward + 2
's' : forward - 2
'f' : lateral, forward, yaw = 0, 0, 0
'k' : yaw = - 1
'l' : yaw = + 1
'z' : depth - 0.3
'x' : depth + 0.3
'c'  : for setting absolute depth of the sub (manual input)
't' : only applicable for Onyx, fires the torpedo
'u' : for moving forward by a certain distance (manual input)
'b' : only applicable for Onyx, drops markers (balls)
'q' : stops movement of the sub and disarms the sub
'm' : stops movement of the sub

'keyboardInterrupt' (Ctrl + C) : stops movement of the sub, disarms the sub
"""
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
