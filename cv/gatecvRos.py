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

camPub = rospy.Publisher('/auv/camera/videoOutput0', Image , queue_size=10)
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

rospy.Subscriber("/auv/camera/videoRaw0",Image,callbackForward) # subscribing here

def navigationPublishing(pwm):
    navPub.publish(pwm)

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
        signal.signal(signal.SIGINT, onExit)
        # if frame is read correctly ret is True
        frame = cv.resize(frame,(480, 270))

        frame = cv.GaussianBlur(frame, (7,7), 0)

        # define the alpha and beta 
        alpha = 1.5 # Contrast control
        beta = 10 # Brightness control

        # call convertScaleAbs function
        adjusted = cv.convertScaleAbs(frame, alpha=alpha, beta=beta)

        #color detection of red: bgr values need tuning
        #rn no color detection is being used:
        lower_red = np.array([0, 0, 50], dtype = "uint8")
        upper_red= np.array([100, 50, 150], dtype = "uint8")
        mask = cv.inRange(frame, lower_red, upper_red)
        redLegs = cv.bitwise_and(frame, frame , mask =  mask)

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        #ALL COLOR DETECTION BYPASSED:
        redLegs = frame

        edges = cv.Canny(redLegs, 100, 200)
        # Display the resulting frame
        cnts, heir = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        #img = cv.drawContours(img, cnts, -1, (0,255,0), 1)
        #contour boxing
        topGateLines = []
        topgateExists = 0
        lines_list =[]
        lines = cv.HoughLinesP(
                    edges, # Input edge image
                    1, # Distance resolution in pixels
                    np.pi/180, # Angle resolution in radians
                    threshold=50, # Min number of votes for valid line
                    minLineLength=10, # Min allowed length of line
                    maxLineGap=50 # Max allowed gap between line for joining them
                    )

        # Iterate over points
        for points in lines:
                # Extracted points nested in the list
            x1,y1,x2,y2=points[0]
            # Draw the lines joing the points
            # On the original image
            slope = ((y2 - y1)/(x2 - x1))
            if slope <= 0.5 and slope >= -0.5:
                cv.line(frame,(x1,y1),(x2,y2),(0, 255, 255),2)
                topGateLines.append([(x1,y1),(x2,y2)])
                topgateExists = 1
                break
            else:
                cv.line(frame,(x1,y1),(x2,y2),(0, 0 , 255),2)
                # Maintain a simples lookup list for points
                lines_list.append([(x1,y1),(x2,y2)])

        # Save the result image
        # cv.imshow('detected lines', frame)

        '''seqNum = 0
        for i in range(0, len(topGateLines)):    
            sum += float(topGateLines[i][0][0])
            sum += float(topGateLines[i][1][0])
            i = seqNum'''
        
        # need to change this to a mean
        if topgateExists == 1:
            mid = (float(topGateLines[0][0][0]) + float(topGateLines[0][1][0]) ) / 2 
            print(mid)
            cv.line(redLegs, (int(mid), 1), (int(mid), 400), (255, 255, 255), thickness= 2)
        
        legs = []
        for cnt in cnts:
            rect = cv.minAreaRect(cnt)
            #print(str(rect))

            box = cv.boxPoints(rect)
            box = np.intp(box)

            width = rect[1][1]
            height = rect[1][0]
            angleOfRotation = rect[2]
            #alternate method : if (width*5) < (height)
            #we are attempting to determine the legs from everything else using a width/height ratio
            #"coefficient of height"
            ch = 0.3
            if (height*ch) > width and height > 50 and angleOfRotation >70 and angleOfRotation < 110:
                #redLegs = cv.drawContours(redLegs,[box],0,(0,255,0),1)
                print(str(height))
                x,y = box[0]
                legs.append((x,width*height))
                #add a tuple containing the x value of the corner as well as the contour area

            #elif (width*0.3) > height and width > 30 and angleOfRotation < 10:
                #redLegs = cv.drawContours(redLegs,[box],0,(255, 255 ,0),1)

            #if not a leg, outline yellow
            else:
                pass
                #redLegs = cv.drawContours(redLegs,[box],0,(0,255,255),2)

        print(lines_list)
        sum = 0
        if len(legs)!=0:
            for tpt in legs:
                sum+=tpt[0]
            print(sum/len(legs))
            print(len(legs))
            #find the average of the x values to find the cneter of the gate
            for i in range(270):
                redLegs[i][int(sum/len(legs))] = [255,255,255]
                redLegs[i][240] = [255,255,0]
            angleVal = 90-round(math.acos(((sum/len(legs))-240)*math.cos(90-69/2)/240)*180/math.pi)
            redLegs = cv.putText(redLegs,str(angleVal),(int(sum/len(legs)),50),cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2,cv.LINE_AA)

        if slope < 0:
            putText(redLegs,250, 250, 'TURN RIGHT' )
            print("yaw clockwise")
            a = 18*[1500]
            a[3] = 1550
        elif slope > 0:
            putText(redLegs, 250, 250, 'TURN LEFT')
            print("yaw counter-clockwise")
            a = 18*[1500]
            a[3] = 1450
        else:
            putText(redLegs,250, 250, 'GG' )
            
            if mid > 240:
                putText(redLegs, 250, 300, 'strafe left')
                print('strafe left')
                a = 18*[1500]
                a[5] = 1420                
            elif mid < 240:
                putText(redLegs, 250, 300, 'strafe right')
                print("strafe right")
                a = 18*[1500]
                a[5] = 1580
            else:
                putText(redLegs, 250, 300, 'go forward')
                print("aligned, go forward")
                a = 18*[1500]
                a[4] = 1660
        
        print("a: " + str(a))
        
        camPub.publish(br.cv2_to_imgmsg(frame)) #publishes processed images

            #navPub.publish(angleVal) #publishes the thing to the thing (this probably won't work rn)

        #cv.imshow('frame', redLegs)
        #cv.imshow('contours', edges)

        if cv.waitKey(5) == ord('q'):
            cv.destroyAllWindows()
            break
    except Exception as e:
        print(e)
        pass
