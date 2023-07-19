"""
Script designed to experiment with the Ping360 sonar and collect some data.
"""

import argparse
import logging
import os
import time

from auv.device.sonar import Ping360, io
from auv.utils import arm, disarm

import numpy as np
import cv2

logging.basicConfig(level=logging.INFO)


parser = argparse.ArgumentParser(description="Ping360 data collection script")
parser.add_argument(
    "--device",
    action="store",
    required=False,
    default="/dev/ttyUSB0",
    type=str,
    help="Ping360 serial device",
)
parser.add_argument(
    "--baudrate",
    action="store",
    required=False,
    default=115200,
    type=int,
    help="Ping360 serial baudrate",
)
parser.add_argument(
    "--output",
    action="store",
    required=False,
    default=str(time.time()) + ".txt",
    type=str,
    help="Output file name",
)

# usage example: python ping360.py --device /dev/ttyUSB0 --baudrate 115200 --output sonar.txt

args = parser.parse_args()

step_angle = 1
max_range = 20

# Create a Ping360 object and connect to the Ping360
p = Ping360(
    args.device,
    args.baudrate,
    scan_mode=1,
    angle_range=(150, 250),
    angle_step=step_angle,
    max_range=max_range,
    gain=2,
    transmit_freq=800,
)

d = p.get_device_data()
print(d)

# make a full scan and save it to a file
logging.info("Starting Ping360 full scan")
arm.arm()

while True:
    try:
        start_time = time.time()
        
        # get objstacle detection from scan
        obj = p.get_obstacles()

        # sort the obstacles by distance
        obj = sorted(obj, key=lambda x: x["distance"])

        logging.info("\n".join(obj))

        imcount += 1

    except KeyboardInterrupt:
        break

disarm.disarm()
