import cv2
import numpy as np
import matplotlib.pyplot as plt

cap = cv2.VideoCapture("pvid.MOV")
#bottom_path_transdec_b_2019_07_30_2_provider_vision_Bottom_GigE_compressed.mp4
#bottom_path_SONIA_1_provider_vision_Bottom_GigE_compressed.mp4
while 1: 
    # ret,frame =cap.read() 
    frame = cv2.imread("p1.jpg")
    frame = cv2.resize(frame, (378, 504))
    frame = cv2.flip(frame, 1)

    into_hsv =(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV))
    L_limit=np.array([8, 100, 100]) 
    U_limit=np.array([50, 255, 255]) 

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
            print("aligned, go forward")
            a = 18*[1500]
            a[4] = 1660

        print(a)
        
    cv2.imshow('Contours', frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
