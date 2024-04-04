"""
Serves as an alternative to QGroundControl for accessing RTSP(Real Time Streaming Protocol) video streams
"""
# To parse command line arguments
import argparse

# To interact with the operating system
import os

# OpenCV for reading frames
import cv2
import numpy as np # For numerical operations

# Create an argument parsing object to parse through the argument
argparser = argparse.ArgumentParser(description="RTSCP Client")

"""
Add the argument for specifying RTSP server's IP address and port:

--host : Argument name
action = "store" : Action to take when this argument is encountered (store the value of the argument)
required = False: Argument is not required
default = "192.168.0.102:8554" : Default value if argument is not provided
type = str: Data type of the argument value
help = "RTSCP Server IP: Port" : Help message displayed when user types "-h" or "--help" (to ask for help)
"""
argparser.add_argument(
    "--host",
    action="store",
    required=False,
    default="192.168.0.102:8554",
    type=str,
    help="RTSCP Server IP:Port",
)

# Parse the command line arguments
args = argparser.parse_args()

# Construct the RTSP URL using the host IP address
RTSP_URL = f"rtsp://{args.host}/test"


# H.265 codec
# Create the video capture object from the RTSP_URL
# The CAP_FFMPEG specifies the backend for video capture
cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)

# Buffer size and FPS for video
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FPS, 30)

# Handle exception if RTSP stream is not open/avaliable to see
if not cap.isOpened():
    print("[ERROR] Failed to open RTSP stream")
    exit(1)

# Read/display each frame on the screen
while True:
    _, frame = cap.read() # "_" used instead of "ret" because boolean of whether frame was successfully read does not matter
    cv2.imshow("frame", frame)

    # When key 'q' pressed, exit the stream
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
