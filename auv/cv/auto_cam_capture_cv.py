"""
Description: CV template class
Author: Team Inspiration
"""

# import what you need from within the package

import time

import cv2
import numpy as np


class CV:
    """Template CV class, don't change the name of the class"""


    def __init__(self, **config):
        self.camera = "/auv/camera/videoUSBRaw0"

        self.frame_width = 640
        self.frame_height = 480
        size = (self.frame_width, self.frame_height)

        self.buffer =  cv2.VideoWriter('AutoVideo.mp4', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)
        



    def run(self, frame, target, oakd_data):

        # Write the frame
        self.vid.write(frame)


        """return the video that will need to have vid.release() done to it in the mission file""" 

        return {"lateral": 0, "forward": 0, "end": False, "vid": self.vid}, frame
    


if __name__ == "__main__":


    # Create a CV object with arguments
    cv = CV()

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture(0)

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # run the cv
        result = cv.run(frame, None, None)



        # do something with the result
    


        # debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


