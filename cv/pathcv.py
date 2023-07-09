import numpy as np
import cv2
import time
import sys
import signal
#from robot_control import RobotControl
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# initializing publisher, will output cv image here
br = CvBridge()
pubForward = rospy.Publisher('/auv/camera/videoUSBOutput1', Image,queue_size=10)
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

def onExit(signum, frame):
    try:
        print("\nDisarming and exiting...")
        rospy.signal_shutdown("Rospy Exited")
        exit(1)
    except:
        pass

rospy.Subscriber("/auv/camera/videoUSBRaw1",Image,callbackForward) # subscribing here

width = 640 # width of frame
height = 480 # height of frame
kill = True

while not rospy.is_shutdown():
    try:
        frame = forwardVideo
        into_hsv =(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))
        L_limit=np.array([8, 100, 100]) 
        U_limit=np.array([50, 255, 255]) 
        signal.signal(signal.SIGINT, onExit)
        # L_limit=np.array([5, 25, 50]) 
        # U_limit=np.array([30, 255, 255]) 
    
        # for more range
        # L_limit = np.array([3, 25, 20])
        # U_limit = np.array([50, 255, 255])
    
    
        orange=cv2.inRange(into_hsv,L_limit,U_limit)
    
        kernel = np.ones((5, 5), np.uint8)
        orange = cv2.morphologyEx(orange, cv2.MORPH_OPEN, kernel)
    
        orange = cv2.GaussianBlur(orange, (11,11), 0)
        ret, thresh = cv2.threshold(orange, 230, 255, cv2.THRESH_BINARY)
    
        blur = cv2.blur(thresh, (10,10))
    
        ret2, thresh2 = cv2.threshold(blur, 1, 255, cv2.THRESH_OTSU)
        contours, heirarchy = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        edges = cv2.Canny(image=thresh2, threshold1=100, threshold2=200)
        
        hline = cv2.line(frame, (640, 0), (0, 0), (0, 255, 255), 8)
        if contours:
            c = max(contours, key = cv2.contourArea)
            cv2.drawContours(frame, c, -1, 255, 3)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(frame, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
            
            extLeft = (c[c[:, :, 0].argmin()][0])
            extRight = (c[c[:, :, 0].argmax()][0])
            extTop = (c[c[:, :, 1].argmin()][0])
            extBot = (c[c[:, :, 1].argmax()][0])
            # if l
            # cv2.line(frame, extLeft, (cx, cy), (255, 255, 0), 8)
            # cv2.line(frame, extBot, (cx, cy), (255, 255, 0), 8)
    
            # if not l
            # line1 = cv2.line(frame, extBot, (cx, cy), (255, 255, 0), 8)
            # line2 = cv2.line(frame, extTop, (cx, cy), (255, 255, 0), 8)
                
            rows,cols = frame.shape[:2]
            [vx,vy,x,y] = cv2.fitLine(c, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)
            cv2.line(frame,(cols-1,righty),(0,lefty),(0,255,0),2)
            cv2.line(frame,(righty,cols-1),(lefty, 0),(0,255,0),2)
            slope = (lefty-righty)/(0-(cols-1))
            pslope = -1/slope
            if(pslope < 0.01):
                print("yaw counter-clockwise")
                a = 18*[1500]
                a[3] = 1450
            elif (pslope > 0.01):
                print("yaw clockwise")
                a = 18*[1500]
                a[3] = 1550
            else:
                #strafe left or right to align
                range = [(width/2)-20, (width/2)+20]
                if(range[0] > (cols-1)):
                    print("strafe right")
                    a = 18*[1500]
                    a[5] = 1580
                elif(range[1] < (cols-1)):
                    print("strafe left")
                    a = 18*[1500]
                    a[5] = 1420
                else:
                    print("aligned, go forward")
                    a = 18*[1500]
                    a[4] = 1660
    
            print(a)
            
        pubForward.publish(br.cv2_to_imgmsg(frame))
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
    except Exception as e:
        print(e)
        pass


