"""
Alternative to QGroundControl for rtsp
"""
import argparse

import os

import cv2
import numpy as np


argparser = argparse.ArgumentParser(description="RTSCP Client")
argparser.add_argument(
    "--host",
    action="store",
    required=False,
    default="192.168.0.102:8554",
    type=str,
    help="RTSCP Server IP:Port",
)

args = argparser.parse_args()

RTSP_URL = f"rtsp://{args.host}/test"
# H265 codec
cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("[ERROR] Failed to open RTSP stream")
    exit(1)

while True:
    _, frame = cap.read()
    cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
