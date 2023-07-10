import numpy as np
import cv2
import time
import sys
#from robot_control import RobotControl
import rospy
import servos as servo



servo.torpedo(1) # shoots first torpedo
torpedo1 = False  
time.sleep(1)
servo.torpedo(2) # shoots second torpedo
torpedo2 = False
time.sleep(1)