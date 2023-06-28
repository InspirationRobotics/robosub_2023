import numpy as np
import cv2
import time
import sys
#from robot_control import RobotControl
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# initializing publisher, will output cv image here
br = CvBridge()
pubForward = rospy.Publisher('/auv/camera/videoOutput0', Image,queue_size=10)
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

rospy.Subscriber("/auv/camera/videoRaw0",Image,callbackForward) # subscribing here

#cap = cv2.VideoCapture('red3.mp4')
# Establishing initial variables
rec = 0
confidence = []
x_left = []
x_center = []
x_right = []
# Start a while loop
#width  = cap.get(3)  # float `width`
#height = cap.get(4)
width = 640 # width of frame
height = 480 # height of frame

# align function, given specific leg, will send "Strafe right" and "Strafe left" commands to align to that leg
def align(rec, confidence, leg):
    pwms = [1500]*18
    if(rec % 5 == 0):
        list = []
        if(confidence.count("Left") > 1):
            list.append("Left")
            sum = 0
            for i in x_left:
                sum+=i

            l_avg = sum/len(x_left)
            if(leg == "Left"):
                if(abs(l_avg-(width/2)) > 50):
                    if(l_avg > (width/2)):
                        print("Strafe right to align to align with left")
                    elif(l_avg < (width/2)):
                        print("Strafe left to align to align with left")
        if(confidence.count("Center") > 1):
            list.append("Center")
            sum = 0
            for i in x_center:
                sum+=i
            c_avg = sum/len(x_center)
            if(leg == "Center"):
                if(abs(c_avg-(width/2)) > 50):
                    if(c_avg > (width/2)):
                        print("Strafe right to align to align with center")
                    elif(c_avg < (width/2)):
                        print("Strafe left to align to align with center")

        if(confidence.count("Right") > 1):
            list.append("Right")
            sum = 0
            for i in x_right:
                sum+=i
            r_avg = sum/len(x_right)
            if(leg == "Right"):
                if(abs(r_avg-(width/2)) > 50):
                    if(r_avg > (width/2)):
                        print("Strafe right to align with right")
                    elif(r_avg < (width/2)):
                        print("Strafe left to align with right")

        c_avg = 0
        r_avg = 0
        l_avg = 0

# Main while loop
while(1):
    try:
        rec+=1
        align(rec, confidence, "Left") # input for which leg to align to
        if(rec % 20 == 0):
            confidence=[]
    # Reading the video from the
    # webcam in image frames
    #_, imageFrame = cap.read()
        imageFrame = forwardVideo
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) # converting image to HSV

	# HSV lower and upper limits for the color red
        red_lower = np.array([120, 50, 50], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
  
        kernel = np.ones((5, 5), "uint8")
      
    # For red color
        red_mask = cv2.dilate(red_mask, kernel)
        red_mask = cv2.GaussianBlur(red_mask, (21, 21), 0) # gaussian blur to get rid of noise
        res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)
    
   
    # Creating contour to track red color
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # detecting contours
        bbox_list = []

	# looping through contours and giving pixel size and height to width ratio requirements
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            ar = float(w)/h
            if(area > 230 and ar<0.45): 
                bbox_list.append((x, y, w, h))

        o_list = []
	# main point of this block of code is to confidently determine which legs are visible which will be fed into the align function
        if(len(bbox_list) >= 2):
            num = len(bbox_list)
            x_list=[]
            for bbox in bbox_list:
                x, y, w, h = bbox
                x_list.append(x)
        
            x_list = sorted(x_list)
            if(len(x_list) > 3):
                x_list = x_list[1:4]

            for bbox in bbox_list:
                x, y, w, h = bbox
                ar = float(w)/h
                o = ""
                if(x in x_list):
                    if(x == x_list[0] and ar < 0.25):
                        o = "Left"
                        confidence.append(o)
                        o_list.append(o)
                        x_left.append(x)
                    elif((x == x_list[-1] and ar < 0.25) and ("Right" not in o_list)):
                        o = "Right"
                        confidence.append(o)
                        o_list.append(o)
                        x_right.append(x)
                    elif(0.45>ar>0.25):
                        if((("Left" in o_list) or ("Right" in o_list)) and ("Center" not in o_list)):
                            o = "Center"
                            confidence.append(o)
                            o_list.append(o)
                            x_center.append(x)

                    area = w*h
                    image = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(image, str(area), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)

    #cv2.imshow("Red", imageFrame)
        pubForward.publish(br.cv2_to_imgmsg(imageFrame)) # publishing video output to /auv/camera/videoForwardOutput0
	#time.sleep(0.05)
        if cv2.waitKey(10) & 0xFF == ord('q'): # code kill
            #cap.release()
            cv2.destroyAllWindows()
            break
    except Exception as e:
        print(e)
        pass
