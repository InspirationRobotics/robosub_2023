"""
Description: CV template class
Author: Team Inspiration
"""

# import what you need from within the package

import logging
import time

import cv2
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class CV:
    """Template CV class, don't change the name of the class"""

    camera = "/auv/camera/videoUSBRaw0"

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """

        self.frame = None
        logger.info("Template CV init")

    def run(self, frame):
        """
        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        logger.info("Template CV run")
        time.sleep(0.041)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply a Gaussian blur to reduce noise
        # blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        # # Apply a morphological operation to close gaps between edge segments
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Detect lines using HoughLinesP
        lines = cv2.HoughLinesP(closed, 1, np.pi/180, threshold=100, minLineLength=50, maxLineGap=20)

        # Draw the detected lines on the original image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2-y1)/(x2-x1)
                if(abs(slope)>20):
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

        self.frame = frame
        return {"lateral": 0, "forward": 0, "end": False}, self.frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template"
    logging.basicConfig(level=logging.INFO)

    # Create a CV object with arguments
    cv = CV(arg1="value1", arg2="value2")

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("GATE.mkv")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # run the cv
        result = cv.run(frame)

        # do something with the result
        frame = result[-1]

        # debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
