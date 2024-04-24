"""
Template method for capturing a camera stream, processing the stream, showing the current frame (at 10 FPS) on the screen, and storing the stream into a video file.
"""

import time

import cv2 # OpenCV library for CV functionality
import numpy as np # For numerical operations


class CV:
    """
    Template CV class, DO NOT change the name of the class. Doing so would mess up all of the backend CV script execution files.
    """

    def __init__(self, **config):
        self.camera = "/auv/camera/videoUSBRaw0" # Default camera source

        # Setting frame dimensions
        self.frame_width = 640
        self.frame_height = 480
        size = (self.frame_width, self.frame_height)

        # Initializing video writer object -- this will store the camera stream in a video file (.mp4) at 10 FPS
        self.buffer =  cv2.VideoWriter('AutoVideo.mp4', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)
        
    def run(self, frame, target, oakd_data):
        """
        Run the CV script

        Args:
            frame: Current frame from camera stream
            target: Target to look for
            oakd_data: List of detections from the ML model; this is only applicable for the OAK-D camera streams
        """
        # Write the frame -- this will write the current frame to the video file
        self.vid.write(frame)

        """
        NOTE: return the video that will need to have vid.release() done to it in the mission file
        """ 

        # Return movement commands, state of the frame, the video containing the camera stream, and the visualized frame.
        # NOTE: The video will need to have vid.release() done to it in the relevant mission file -- this is essential to cleanly exit
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


