from math import radians
import cv2
import numpy as np
import time

img = cv2.imread('gate.png')
img = cv2.resize(img, (1000, 462))

img = cv2.GaussianBlur(img, (5,5), 0)

#color detection of red: bgr values need tuning
lower_red = np.array([0, 0, 50], dtype = "uint8") 
upper_red= np.array([100, 50, 150], dtype = "uint8")
mask = cv2.inRange(img, lower_red, upper_red)
redLegs = cv2.bitwise_and(img, img , mask =  mask) 
#cv2.imshow("red color detection", redLegs) 

#more image processing
gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

hsv= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#while True:
#k = cv2.waitKey(1) & 0xFF
#if k == 27:
#    break

#detected_output = cv2.bitwise_and(img, img , mask =  mask)

edges = cv2.Canny(redLegs, 100, 200)

cnts, heir = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#img = cv2.drawContours(img, cnts, -1, (0,255,0), 1)
#contour boxing
for cnt in cnts:
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    width = rect[1][1]
    height = rect[1][0]
    if (width) * (height) >= 30:
        print('width is: ' + str(width))
        print('height is: ' + str(height))
        redLegs = cv2.drawContours(redLegs,[box],0,(0,255,255),2)

    else:
        redLegs = cv2.drawContours(redLegs,[box],0,(0,255,0),2)

    
    y,x = box[0]
    print(box[0])
    redLegs[x][y]=(0,0,0)

    #determine height and width of the rectangles

cv2.imshow('source', img)
cv2.imshow('edges', edges)
cv2.imshow('image', redLegs)
cv2.waitKey()

#time.sleep(100)
#cv2.imshow('gray', gray)

#cv2.imshow('gr',gray)
    
#cv2.destroyAllWindows()
