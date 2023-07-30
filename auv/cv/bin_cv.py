"""
Description: Bin CV
Author: Raymond Song
"""

# import what you need from within the package

import time

import cv2
import numpy as np


class CV:
    """Template CV class, don't change the name of the class"""

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dictionnary containing the config of the sub
        """
        self.config = config
        self.current_sub = self.config.get("sub", "onyx")
        if self.current_sub == "onyx":
            self.camera = "/auv/camera/videoOAKdRawBottom"
            self.model = "bins3"
        elif self.current_sub == "graey":
            print(f"[INFO] No Gripper or Dropper on graey")
            self.camera = None

        self.viz_frame = None

        print("[INFO] Bin CV init")

    def run(self, frame, target, oakd_data):
        """
        frame: the frame from the camera
        target: could be any type of information, for example the thing to look for
        oakd_data: only applies for oakd cameras, this is the list of detections

        Here should be all the code required to run the CV.
        This could be a loop, grabing frames using ROS, etc.
        """
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0
        drop = False

        height, width, _ = frame.shape

        tolerance = 10
        maxConfidence = 0 
        target_bin = None

        # measured offset from the footage
        target_pixel = (190, 300)

        if oakd_data == None:
            return {}, frame

        for detection in oakd_data:
            x1 = int(detection.xmin)
            x2 = int(detection.xmax)
            y1 = int(detection.ymin)
            y2 = int(detection.ymax)

            if target in detection.label:
                centerOfDetection = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                
                # Get the highest confidence
                if detection.confidence > maxConfidence:
                    maxConfidence = detection.confidence
                    target_bin = centerOfDetection

        cv2.circle(frame, target_pixel, 10, (0, 255, 0), -1)
        # go slowly foward until we see the target
        if target_bin is None:
            return {"forward": 0.8}, frame
        cv2.circle(frame, target_bin, 10, (0, 0, 255), -1)

        x_error = (target_bin[0] - target_pixel[0]) / width
        y_error = (target_pixel[1] - target_bin[1]) / height

        # apply a gain and clip the values
        lateral = np.clip(x_error * 3, -1, 1)
        forward = np.clip(y_error * 3, -1, 1)
        
        if abs(x_error) < tolerance / width and abs(y_error) < tolerance / height:
            drop = True

        # TODO (low priority): Remove Lids for each bin
        return {"lateral": lateral, "forward": forward, "drop": drop}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template_cv"

    # Create a CV object with arguments
    cv = CV()

    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("../../testing_data/")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # run the cv
        result = cv.run(frame, "some_info", None)

        # do something with the result
        print(f"[INFO] {result}")

        # debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
