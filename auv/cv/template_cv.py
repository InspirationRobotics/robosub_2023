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
        logging.info("Template CV run")

        return {"lateral": 0, "forward": 0}, self.frame



if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template"

    # Create a CV object with arguments
    cv = CV(arg1="value1", arg2="value2")

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture(0)

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # run the cv
        result = cv.run(frame)

        # do something with the result
        logger.info(result)

        # debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
