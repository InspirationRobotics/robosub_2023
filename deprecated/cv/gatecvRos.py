import numpy as np
import cv2 as cv
import time
import sys
import math
import signal
import mavros_msgs.msg
import mavros_msgs.srv
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# initializing publisher, will output cv image here
br = CvBridge()

camPub = rospy.Publisher('/auv/camera/videoUSBOutput0', Image , queue_size=10)
navPub = rospy.Publisher('/auv/devices/thrusters', mavros_msgs.msg.OverrideRCIn, queue_size=10)

global forwardVideo
forwardVideo = None
rospy.init_node("CV", anonymous=True)
rospy.Rate(30)

# initializing subscriber and callback to retrieve video feed, assigning to forwardVideo 
def callbackForward(msg):
        global forwardVideo
        try:
            forwardVideo = br.imgmsg_to_cv2(msg)
        except Exception as e:
            print("Forward Output Error, make sure running in Python2")
            print(e)

rospy.Subscriber("/auv/camera/videoUSBRaw0",Image,callbackForward) # subscribing here

def navigationPublishing(pwm):
    #navPub.publish(pwm)
    print(pwm)

def onExit(signum, frame):
    try:
        print("\nDisarming and exiting...")
        rospy.signal_shutdown("Rospy Exited")
        exit(1)
    except:
        pass

def putText(pic, x, y, text):
    pic = cv.putText(pic,str(text),(x,y),cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2,cv.LINE_AA)
    

#cap = cv.VideoCapture('gate.mkv')
#cap = cv.VideoCapture('gate.mkv')

 
while not rospy.is_shutdown():
    try:
        frame = forwardVideo
        f= frame
        print(f)
        signal.signal(signal.SIGINT, onExit)
        # if frame is read correctly ret is True
        #frame = cv.resize(frame,(480, 270))

                
        camPub.publish(br.cv2_to_imgmsg(f)) #publishes processed images

            #navPub.publish(angleVal) #publishes the thing to the thing (this probably won't work rn)

        #cv.imshow('frame', redLegs)
        #cv.imshow('contours', edges)

        if cv.waitKey(5) == ord('q'):
            cv.destroyAllWindows()
            break
    except Exception as e:
        print(e)
        pass
