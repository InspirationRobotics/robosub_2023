from math import radians
import cv2 as cv
import numpy as np
import time

#determines the theta value, or the angle of the sub relative to the gate
def findTheta(h1, h2):
    hdiff = h1-h2
    if hdiff > 0: 
        print('left')
    elif hdiff < 0:
        print('right')
    elif hdiff == 0:
        print('congrats')

#determines the distance between the two legs in the camera feed
def gateCompression(l1, l2):
    return(abs(l1-l2))

def legRatio(h1, h2):
    return(h1/h2)


img = cv.imread('gate1.png')
gateFeed = cv.VideoCapture('gate.mkv')

img = cv.resize(img, (1000, 462))

img = cv.GaussianBlur(img, (5,5), 0)

#color detection of red: bgr values need tuning
lower_red = np.array([0, 0, 50], dtype = "uint8")
upper_red= np.array([100, 50, 150], dtype = "uint8")
mask = cv.inRange(img, lower_red, upper_red)
redLegs = cv.bitwise_and(img, img , mask =  mask)
#cv.imshow("red color detection", redLegs) 

#more image processing
gray= cv.cvtColor(img,cv.COLOR_BGR2GRAY)

hsv= cv.cvtColor(img,cv.COLOR_BGR2HSV)
#while True:
#k = cv.waitKey(1) & 0xFF
#if k == 27:
#    break

#detected_output = cv.bitwise_and(img, img , mask =  mask)

edges = cv.Canny(redLegs, 100, 200)

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

    #determine height and width of the rectangles

cv.imshow('source', img)
cv.imshow('edges', edges)
cv.imshow('image', redLegs)

cv.waitKey()

#time.sleep(100)
#cv.imshow('gray', gray)

#cv.imshow('gr',gray)
    
#cv.destroyAllWindows()
