"""
Template class for capturing, reading a frame, and doing CV (computer vision) manipulations
"""

import time

import cv2 # OpenCV library for CV functionality
import numpy as np # For numerical operations


class CV:
    """Template CV class, don't change the name of the class"""

    def __init__(self, **config):
        self.camera = "/auv/camera/videoUSBRaw0" # Default camera source

        # Setting frame dimensions
        self.frame_width = 640
        self.frame_height = 480
        size = (self.frame_width, self.frame_height)

        # Initializing video writer object
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

    # Initializing camera capture object
    cap = cv2.VideoCapture(0)

    while True:
        # Grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # Run the CV processing
        result = cv.run(frame, None, None)



        #  Do something with the result (will be mission-specific)
    


        # Show the frame for debugging purposes
        cv2.imshow("frame", frame)

        # If key 'q' pressed, then exit the program
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


