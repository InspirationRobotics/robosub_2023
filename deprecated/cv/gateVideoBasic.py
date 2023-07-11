import cv2 as cv
import numpy as np
import time

cap = cv.VideoCapture('gate.mkv')

if not cap.isOpened():
 print("Cannot open camera")
 exit()
while True:
 # Capture frame-by-frame
 ret, frame = cap.read()
 # if frame is read correctly ret is True
 frame = cv.resize(frame,(480, 270))
 
 frame = cv.GaussianBlur(frame, (5,5), 0)

 #color detection of red: bgr values need tuning
 lower_red = np.array([0, 0, 50], dtype = "uint8")
 upper_red= np.array([100, 50, 150], dtype = "uint8")
 mask = cv.inRange(frame, lower_red, upper_red)
 redLegs = cv.bitwise_and(frame, frame , mask =  mask)

 #if not ret:
   # print("Can't receive frame (stream end?). Exiting ...")
    #break  
 # Our operations on the frame come here

 gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

 edges = cv.Canny(redLegs, 100, 200)
 # Display the resulting frame
 cnts, heir = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
#img = cv.drawContours(img, cnts, -1, (0,255,0), 1)
#contour boxing
 for cnt in cnts:
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)

    width = rect[1][1]
    height = rect[1][0]
    if (width) * (height) >= 30:
        print('width is: ' + str(width))
        print('height is: ' + str(height))
        redLegs = cv.drawContours(redLegs,[box],0,(0,255,255),2)

    else:
        redLegs = cv.drawContours(redLegs,[box],0,(0,255,0),2)

    
    y,x = box[0]
    print(box[0])
    redLegs[x][y]=(0,0,0)

 cv.imshow('frame', redLegs)

 if cv.waitKey(5) == ord('q'):
   break
# When everything done, release the capture
cap.release()