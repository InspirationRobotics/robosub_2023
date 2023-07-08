"""
Script designed to experiment with the Ping360 sonar and collect some data.
"""

import argparse
import logging
import os
import time

from sonar import Ping360
from sonar.io import Record
from sonar import utils

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

# Create a Ping360 object and connect to the Ping360
p = Ping360(
    args.device,
    args.baudrate,
    scan_mode=0,
    angle_range=(0, 399),
    angle_step=2,
)


p.set_transmit_frequency(800)
p.set_sample_period(600)  # 25ns units : 400*25ns = 10us
p.set_number_of_samples(1000)
p.set_gain_setting(2)

d = p.get_device_data()
print(d)

# make a full scan and save it to a file
logging.info("Starting Ping360 full scan")
r = Record(args.output, "w")

imsize = 400
img = np.zeros((imsize, imsize, 3), dtype=np.uint8)

imcount = 0

while True:
    try:
        start_time = time.time()

        for ts, angle, points in p:
            r.write(ts, angle, points)
            utils.plot_to_polar_color(img, angle, points, imsize=imsize)

        end_time = time.time()

        cartesian = utils.polar_to_cart(img)
        cv2.imwrite(str(imcount) + "_cart.png", cartesian)
        cv2.imwrite(str(imcount) + "_polar.png", img)

        logging.info(f"Full scan complete in {end_time - start_time} seconds")

        imcount += 1

    except KeyboardInterrupt:
        break
