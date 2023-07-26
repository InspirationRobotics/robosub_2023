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
        print("[INFO] Bin CV run")
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0
        drop = False

        # TODO: need to figure out how/when to end the cv
        end = False 

        tolerance = 10
        maxConfidence = 0
        bin_offset = (0, 0) # measured offset
        target_bin = (0, 0)
        target_pixel = (320, 240)

        if oakd_data == None:
            return {}, frame

        for detection in oakd_data:
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)

            label = detection.label
            cv2.putText(frame, str(detection.label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"{detection.confidence * 100:.2f}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

            # Check Abydos
            # TODO: map the class names in the .json of the model to the actual names
            if detection.label == target:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), cv2.FONT_HERSHEY_SIMPLEX)
                centerOfDetection = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                
                # Get the highest confidence
                if detection.confidence > maxConfidence:
                    maxConfidence = detection.confidence
                    target_bin = centerOfDetection

        cv2.circle(frame, target_bin, 10, (0, 0, 255), -1)

        # align X
        if target_bin[0] < target_pixel[0] + bin_offset[0] - tolerance:
            # strafe right
            lateral = 1.5
        elif target_bin[0] > target_pixel[0] + bin_offset[0] + tolerance:
            # strage left
            lateral = -1.5
        else:
            # align Y
            if target_bin[1] < target_pixel[1] + bin_offset[1] - tolerance:
                forward = -1
            elif target_bin[1] > target_pixel[1] + bin_offset[1] + tolerance:
                forward = 1
            else:
                # Drop the ball
                drop = True

        # TODO: Remove Lids for each bin

        return {"lateral": lateral, "forward": forward, "vertical": vertical, "end":end, "drop": drop}, frame


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template_cv"

    # Create a CV object with arguments
    cv = CV(arg1="value1", arg2="value2")

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
