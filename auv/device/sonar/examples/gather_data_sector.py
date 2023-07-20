"""
Script designed to experiment with the Ping360 sonar and collect some data.
"""

import argparse
import os
import time

import cv2
import numpy as np

from auv.device.sonar import Ping360, io, utils

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
    default=f"{str(time.time())}.txt",
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
print("[INFO] Starting Ping360 full scan")
r = io.Record(args.output, "w")

size = (400, p._number_of_samples)
img = np.zeros((size[0], size[1], 1), dtype=np.uint8)

imcount = 0

while True:
    try:
        start_time = time.time()

        for ts, angle, points in p:
            print(angle)
            r.write(ts, angle, points)
            utils.plot_to_polar_gray(
                img,
                angle,
                points,
                imsize=size,
                step_angle=step_angle,
            )

        end_time = time.time()

        cartesian = utils.polar_to_cart(img)
        cv2.imwrite(f"{str(imcount)}_cart.png", cartesian)
        cv2.imwrite(f"{str(imcount)}_polar.png", img)

        print(f"[INFO] Full scan complete in {end_time - start_time} seconds")

        imcount += 1

    except KeyboardInterrupt:
        break
