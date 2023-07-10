import numpy as np
import cv2
import time
import sys
#from robot_control import RobotControl
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

br = CvBridge()
pubForward = rospy.Publisher('/auv/camera/videoOutput0', Image,queue_size=10)
global forwardVideo
forwardVideo = None
#rospy.init_node("MarkerCV", anonymous=True)
rospy.Rate(30)

def callbackForward(msg):
        global forwardVideo
        try:
            forwardVideo = br.imgmsg_to_cv2(msg)
        except Exception as e:
            print("Forward Output Error, make sure running in Python2")
            print(e)

rospy.Subscriber("/auv/camera/videoRaw0",Image,callbackForward)

while 1: 
    try:
        frame = forwardVideo
        into_hsv =(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))
        L_limit=np.array([0, 20, 20]) 
        U_limit=np.array([30, 255, 150]) 
    
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Apply Canny edge detection
    
        orange=cv2.inRange(into_hsv,L_limit,U_limit)
    
        kernel = np.ones((5, 5), np.uint8)
          
        # For red color
        orange = cv2.dilate(orange, kernel)
        orange = cv2.GaussianBlur(orange, (21, 21), 0)
        
        ret, thresh = cv2.threshold(orange, 230, 255, cv2.THRESH_BINARY)
    
        blur = cv2.blur(thresh, (10,10))
    
        ret2, thresh2 = cv2.threshold(blur, 1, 255, cv2.THRESH_OTSU)
    
       # edges = cv2.Canny(image=into_hsv, threshold1=100, threshold2=200)
       # mask = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    
      #  image_with_edges = cv2.bitwise_and(frame, mask)
    
        contours, heirarchy = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            frame = cv2.drawContours(frame, largest_contour, -1, (0, 255, 0), 2)
        
        pubForward.publish(br.cv2_to_imgmsg(frame))
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
    
    except Exception as e:
        print(e)
        pass
