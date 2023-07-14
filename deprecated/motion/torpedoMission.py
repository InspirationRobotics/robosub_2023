import numpy as np
import cv2
import time
import sys
import signal
import mavros_msgs.msg
import mavros_msgs.srv
import numpy as np
import cv2
from robot_control import RobotControl
import time
import os
from servos import servos
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

br = CvBridge()
pubForward = rospy.Publisher('/auv/camera/videoUSBOutput0', Image,queue_size=10)
global forwardVideo
pubThrusters = rospy.Publisher('/auv/devices/thrusters', mavros_msgs.msg.OverrideRCIn, queue_size=10)
forwardVideo = None
rospy.init_node("CV", anonymous=True)
rospy.Rate(30)
servo = servos()


# initializing subscriber and callback to retrieve video feed, assigning to forwardVideo 
def callbackForward(msg):
    global forwardVideo
    try:
        forwardVideo = br.imgmsg_to_cv2(msg)
    except Exception as e:
        print("Forward Output Error")
        print(e)

def onExit(signum, frame):
    try:
        print("\nDisarming and exiting...")
        rospy.signal_shutdown("Rospy Exited")
        exit(1)
    except:
        pass

rospy.Subscriber("/auv/camera/videoUSBRaw0",Image,callbackForward) # subscribing here

width = 640 # width of frame
height = 480 # height of frame
kill = True

#cap = cv2.VideoCapture(1)
# cap = cv2.VideoCapture(0)
#cap = forwardVideo

sensitivity1 = 10           # Higher will 
sensitivity2 = 100
lostSight = 0
lastMove = -1           # None = -1, CCW = 1, CW = 2, FWD = 3, 


torpedo1 = True # default torpedoes values are either loaded or unloaded
torpedo2 = True

#rc = RobotControl()

param1 = 1
param2 = 325  # Higher is less lines in edges # Lower is More lines in edges



#Presuming that 0 is far left, 500 is far right
# 250 should be center
# Want biggest radius to be around 180 and that is clsoe enough the the closed iris

while not rospy.is_shutdown():
    try:    # Capture frame-by-frame
        frame = forwardVideo

        if frame is None:
            time.sleep(0.1)
            continue

        #output_frame = frame.copy()
        output_frame = frame

        # frame_bgr = cv2.medianBlur(frame_bgr, 3)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, param1, param2)
        # Blurring may need to be removed
        edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
        edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
        #edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)
        #edges = cv2.GaussianBlur(edges, (5, 5), 2, 2)



        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, edges.shape[0] / 8, param1=sensitivity1, param2=sensitivity2, minRadius=1, maxRadius=600)

        # If we have extracted a circle, draw an outline

        # Need to paint all circles
        if circles is not None:
            print (circles)
            circles = np.round(circles[0, :]).astype("int")
            i = 0
            lostSight = 0
            for circle in circles:
                
                x,y = circles[i,0], circles[i,1]
                dist = circles[i, 2]
                cv2.circle(output_frame, center=(x, y), radius=dist, color=(0, 255, 0), thickness=20)
                cv2.circle(output_frame, center=(circles[i, 0], circles[i, 1]), radius=2, color=(0, 0, 255), thickness=2)
                
                #i += 1
        if circles is None:
            lostSight += 1

        

        # Display the resulting frame, quit with q
        cv2.imshow('frame', output_frame)
        cv2.imshow('edges', edges)

        # Horizontal adjustment on x variable
        if (x < 220 and lostSight < 10):   # Target is on the left side and need to rotate CCW
            if(dist < 250):  # need to rotate CW
                ##rc.setHeading(rc.get_compass - 10)
                #a = 18*[1500]
                #a[3] = 1450
                #time.sleep(0.5)
                #a[3] = 1500
                lastMove = 1
                print("Turning CCW")

            if(dist > 250):
                # Strafe right
                ##rc.lateralUni(-0.5, 0.5)
                
                print("Strafe left")


        if (x > 280 and lostSight < 10):   # Target is on the right
                if(dist < 250):  # need to rotate CW
                    #rc.setHeading(rc.get_compass + 10)
                    #a = 18*[1500]
                    #a[3] = 1550
                    #time.sleep(0.5)
                    #a[3] = 1500
                    lastMove = 2
                    print("Turning CW")

                if(dist > 250):
                    # Strafe right
                    #rc.lateralUni(0.5, 0.5)
                    print("Strafe right")
                    #rc.setHeading(rc.get_compass + 10)
                    #a = 18*[1500]
                    #a[5] = 1580
                    #time.sleep(0.5)
                    #a[5] = 1500
                        
                    lastMove = 2


        if(lostSight):
            print("lost sight")
            if(lastMove == 1):
                #rc.setHeading(rc.get_compass + 2.5)
                print("Lost - goingt CW")
            if(lastMove == 2):
                #rc.setHeading(rc.get_compass - 2.5)
                print("Lost - going CCW")
                


            



        # Vertical adjustment on y variable
            # TODO

        if(dist < 270): # Need to move forward
            #rc.forwardDist(0.5, 1) 
            print("Forward")



        if(dist > 300): # Need to move backwards
            #rc.forwardHeadingUni(-0.5, 1)
            print("Backward")



        time.sleep(1)

        # Conditions to shoot torpedo
        if (dist > 270 and dist < 300) and (x > 220 and x < 270):
            if(torpedo1):
                print("Shooting torpedo 1")
                servo.torpedoLauncher(1) # shoots first torpedo
                torpedo1 = False  
                time.sleep(1)

            if(torpedo2):
                print("Shooting torpedo 2")
                servo.torpedoLauncher(2) # shoots second torpedo
                torpedo2 = False
                time.sleep(1)

        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = a
        print(a)
        pubThrusters.publish(pwm)
        
   
        #frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        pubForward.publish(br.cv2_to_imgmsg(frame))
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
        print("~~~~~~~~~~~~~~~~~~~~~~~~~")


    except Exception as e:
        print(e)
        pass


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
