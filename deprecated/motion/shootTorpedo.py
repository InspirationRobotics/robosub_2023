import sys
import time

import cv2
import numpy as np
import rospy

from auv.motion.servo import Servo

servo = Servo()

servo.torpedo(1) # shoots first torpedo
torpedo1 = False  
time.sleep(1)
servo.torpedo(2) # shoots second torpedo
torpedo2 = False
time.sleep(1)