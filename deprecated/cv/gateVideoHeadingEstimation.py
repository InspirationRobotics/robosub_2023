import cv2 as cv
import numpy as np
import time
import math

cap = cv.VideoCapture("gate.mkv")

if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    frame = cv.resize(frame, (480, 270))

    frame = cv.GaussianBlur(frame, (5, 5), 0)

    # color detection of red: bgr values need tuning
    lower_red = np.array([0, 0, 50], dtype="uint8")
    upper_red = np.array([100, 50, 150], dtype="uint8")
    mask = cv.inRange(frame, lower_red, upper_red)
    redLegs = cv.bitwise_and(frame, frame, mask=mask)

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    edges = cv.Canny(redLegs, 100, 200)
    # Display the resulting frame
    cnts, heir = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    # img = cv.drawContours(img, cnts, -1, (0,255,0), 1)
    # contour boxing
    legs = []
    for cnt in cnts:
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)

        width = rect[1][1]
        height = rect[1][0]
        if (width) * (height) >= 30:
            redLegs = cv.drawContours(redLegs, [box], 0, (0, 255, 255), 2)
            x, y = box[0]
            legs.append((x, width * height))
            # add a tuple containing the x value of the corner as well as the contour area

        else:
            redLegs = cv.drawContours(redLegs, [box], 0, (0, 255, 0), 2)

        y, x = box[0]
        redLegs[x][y] = (0, 0, 0)
    sum = 0
    for tpt in legs:
        sum += tpt[0]
    print(sum / len(legs))
    print(len(legs))
    # find the average of the x values to find the cneter of the gate
    for i in range(270):
        redLegs[i][int(sum / len(legs))] = [255, 255, 255]
        redLegs[i][240] = [255, 255, 0]
    angleVal = 90 - round(math.acos(((sum / len(legs)) - 240) * math.cos(49.5) / 240) * 180 / math.pi)
    redLegs = cv.putText(
        redLegs, str(angleVal), (10, int(sum / len(legs))), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA
    )
    cv.imshow("frame", redLegs)
    # cv.imshow('cnts',edges)
    # cv.imshow("image",frame)
    if cv.waitKey(5) == ord("q"):
        break
# When everything done, release the capture
cap.release()
