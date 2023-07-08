"""
Script to replay data from a file, do processing on those and visualize it.
"""

import argparse
import logging
import os
import time

from sonar import Ping360, utils
from sonar.io import Playback

import numpy as np
import cv2

logging.basicConfig(level=logging.INFO)

parser = argparse.ArgumentParser(description="Ping360 data collection script")
parser.add_argument(
    "--input",
    action="store",
    required=True,
    type=str,
    help="Input file name",
)
args = parser.parse_args()

# Create a Playback object to simulate the Ping360
p = Playback(args.input)

t, a, d = next(p)
size = len(d)

polar_img = np.zeros((400, size, 1), dtype=np.uint8)
cart_img = np.zeros((size, size, 1), dtype=np.uint8)

detection_every = 10
detection_counter = 0

for timestamp, angle, data in p:
    # Convert the data to a numpy array
    data = list(data)

    # Convert the data to a polar image
    polar_img = utils.plot_to_polar_gray(
        polar_img,
        angle,
        data,
        imsize=size,
    )

    # Convert the polar image to a cartesian image
    cart_img = utils.polar_to_cart(polar_img, imsize=size)

    # detect obstacles
    detection_counter += 1
    if detection_counter == detection_every: 
        obstacles = utils.object_detection(polar_img, threshold=80)

        # draw obstacles
        obs_img = cv2.fillPoly(np.zeros_like(polar_img), [obs.points for obs in obstacles], 255)
        cv2.imshow("Obstacles", obs_img)
        detection_counter = 0

    cv2.imshow("Polar", polar_img)
    cv2.imshow("Cart", cart_img)
    cv2.waitKey(1)
