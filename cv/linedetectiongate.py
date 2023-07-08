import cv2 as cv
import numpy as np
import time

frame = cv.imread('gate1.PNG')

frame = cv.resize(frame,(960, 540))

frame = cv.GaussianBlur(frame, (5,5), 5)

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

#ALL COLOR DETECTION BYPASSED:
redLegs = frame

gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

edges = cv.Canny(redLegs, 100, 25)
# Display the resulting frame
cnts, heir = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
#img = cv.drawContours(img, cnts, -1, (0,255,0), 1)
#contour boxing
lines_list =[]
lines = cv.HoughLinesP(
            edges, # Input edge image
            1, # Distance resolution in pixels
            np.pi/180, # Angle resolution in radians
            threshold=100, # Min number of votes for valid line
            minLineLength=10, # Min allowed length of line
            maxLineGap=50 # Max allowed gap between line for joining them
            )
  
# Iterate over points
for points in lines:
      # Extracted points nested in the list
    x1,y1,x2,y2=points[0]
    # Draw the lines joing the points
    # On the original image
    cv.line(frame,(x1,y1),(x2,y2),(0, 0 , 255),2)
    # Maintain a simples lookup list for points
    lines_list.append([(x1,y1),(x2,y2)])
      
# Save the result image
cv.imshow('detected lines', frame)

legs = []
for cnt in cnts:
    rect = cv.minAreaRect(cnt)
    #print(str(rect))

    box = cv.boxPoints(rect)
    box = np.intp(box)

    width = rect[1][1]
    height = rect[1][0]

    #alternate method : if (width*5) < (height)
    #we are attempting to determine the legs from everything else using a width/height ratio
    #"coefficient of height"
    ch = 0.3
    if (height*ch) > width and height > 200  :
        redLegs = cv.drawContours(redLegs,[box],0,(0,255,0),2)
        print(str(height))
        x,y = box[0]
        legs.append((x,width*height))
        #add a tuple containing the x value of the corner as well as the contour area

    #if not a leg, outline yellow
    else:
        redLegs = cv.drawContours(redLegs,[box],0,(0,255,255),2)

cv.imshow('frame', redLegs)
cv.imshow('contours', edges)

print(lines_list)

cv.waitKey()