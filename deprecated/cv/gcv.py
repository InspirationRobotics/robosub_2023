import numpy as np
import cv2
import time

cap = cv2.VideoCapture('short.mp4')
rec = 0
confidence = []
x_left = []
x_center = []
x_right = []
# Start a while loop
width  = cap.get(3)  # float `width`
height = cap.get(4)

while(1):
    rec+=1
    # Reading the video from the
    # webcam in image frames
    _, imageFrame = cap.read()

    imageFrame = cv2.resize(imageFrame, (640, 480))
    time.sleep(0.041)
    gray = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blurred, 50, 150, apertureSize=3)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    ret, thresh1 = cv2.threshold(edges, 120, 255, cv2.THRESH_BINARY)
    # For red color
    cv2.imshow("edges", thresh1)
    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(imageFrame, contours, -1, (0, 255, 0), 3)
    # bbox_list = []
    
    # for pic, contour in enumerate(contours):
    #     area = cv2.contourArea(contour)
    #     x, y, w, h = cv2.boundingRect(contour)
    #     ar = float(w)/h
    #     if(area > 230 and ar<0.45): 
    #         bbox_list.append((x, y, w, h))
    #         cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)


    cv2.imshow("Red", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break