import cv2 as cv
import numpy as np
import time

cap = cv.VideoCapture('vid1.MP4')

if not cap.isOpened():
 print("Cannot open camera")
 exit()
while True:
 # Capture frame-by-frame
 ret, frame = cap.read()
 orig = frame
 # if frame is read correctly ret is True
 frame = cv.resize(frame,(960, 540))
 
 frame = cv.GaussianBlur(frame, (5,5), 0)

 #color detection of red: bgr values need tuning
 lower_red = np.array([0, 0, 50], dtype = "uint8")
 upper_red= np.array([100, 50, 150], dtype = "uint8")
 mask = cv.inRange(frame, lower_red, upper_red)
 redLegs = cv.bitwise_and(frame, frame , mask =  mask)

 gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

 edges = cv.Canny(gray, 100, 200)
 # Display the resulting frame
 cnts, heir = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
#img = cv.drawContours(img, cnts, -1, (0,255,0), 1)
#contour boxing
 legs = []
 for cnt in cnts:
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)

    width = rect[1][1]
    height = rect[1][0]
    if (width) * (height) >= 30:
        redLegs = cv.drawContours(redLegs,[box],0,(0,255,255),2)
        x,y = box[0]
        legs.append((x,width*height))
        #add a tuple containing the x value of the corner as well as the contour area

    else:
        redLegs = cv.drawContours(redLegs,[box],0,(0,255,0),2)
   
    y,x = box[0]
    redLegs[x][y]=(0,0,0)

 sum = 0
 leftLeg = 10000
 biggerLeg = 0
 leftLegIndex = 0
 biggerLegIndex = 0
 secondBiggerLeg = 0
 secondBiggerLegIndex = 0

 #legs is an array of tuples (x, area) 
 #each time we run through this for loop, we get the next tuple
 #this is assigned to the array tpt
 #after this, we can extract the x values
 for index, leg in enumerate(legs):
     if leg[0] < leftLeg:
        leftLeg = leg[0]
        leftLegIndex = index
     if leg[1] > biggerLeg:
        secondBiggerLeg = biggerLeg
        secondBiggerLegIndex = biggerLegIndex
        biggerLeg = leg[1]
        biggerLegIndex = index
     sum+=leg[0]
 try:
    cv.putText(redLegs, 'left leg', (leftLeg, 20), 1, 1, (0, 255, 0))
    cv.putText(redLegs, 'bigger leg', (legs[biggerLegIndex][0], 10), 1, 1, (0, 0, 255))
    cv.putText(redLegs, 'second biggest leg', (legs[secondBiggerLegIndex][0], 30), 1, 1, (255, 0, 0))
# navigation section: what direction to rotate/strafe based on the computer vision: depending on which leg is larger...
    if legs[biggerLegIndex][0] < (int(legs[secondBiggerLegIndex][0]) *1.05) and (legs[biggerLegIndex][0] > (int(legs[secondBiggerLegIndex][0]) * 0.95)):
        print('go forward')
        cv.putText(redLegs, 'go forward', (300, 300), 1, 1, (0, 255, 0))
    elif legs[biggerLegIndex][0] >= legs[secondBiggerLegIndex][0]:
        print('rotate clockwise')
        cv.putText(redLegs, 'rotate clockwise', (300, 300), 1, 1, (0, 255, 0))
    elif legs[biggerLegIndex][0] < legs[secondBiggerLegIndex][0]:
        print('rotate counterclockwise')
        cv.putText(redLegs, 'rotate counterclockwise', (300, 300), 1, 1, (0, 255, 0))
 
    #print(sum/len(legs))
    #print(len(legs))
    #find the average of the x values to find the cneter of the gate
    midlegs = int(sum/len(legs))
    midframe = (redLegs.shape[:2][0]/2, redLegs.shape[:2][1]/2)

 except:
    print("error")
 
 print(midlegs)
 print(midframe)

 if midlegs*1.05 > int(midframe[1]) and midlegs*3< int(midframe[1])*3:
    print('move forward')
    cv.putText(redLegs, 'move forward', (300, 310), 1, 1, (0, 255, 0))
 elif midlegs > int(midframe[1]):
    print('move left')
    cv.putText(redLegs, 'move left', (300, 310), 1, 1, (0, 255, 0))
 elif midlegs <= int(midframe[1]):
    print('move right')
    cv.putText(redLegs, 'move right', (300, 310), 1, 1, (0, 255, 0))

 for i in range(480):
   redLegs[i][midlegs] = [255,255,255] #draws center of the gate
   redLegs[i][int(midframe[1])] = [255,0,255] # draws center of the frame
 cv.imshow('frame', redLegs)
 cv.imshow('cnts',edges)
 cv.imshow('orig', orig)
 if cv.waitKey(5) == ord('q'):
    break
 time.sleep(0.01)
# When everything done, release the capture
cap.release()