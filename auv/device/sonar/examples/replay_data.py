"""
Script to replay data from a file, do processing on those and visualize it.
"""

import argparse
import os
import time

import cv2
import numpy as np

from auv.device.sonar import Ping360, io, utils

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
p = io.Playback(args.input)

# take the first two scans to get the step_angle and num_samples
t, a0, d = next(p)
t, a1, d = next(p)
num_samples = len(d)
step_angle = a1 - a0

size = (400, num_samples)

polar_img = np.zeros((size[0], size[1]), dtype=np.uint8)
cart_img = np.zeros((size[1], size[1], 1), dtype=np.uint8)

print("step_angle: ", step_angle)
print("num_samples: ", num_samples)
print("size: ", size)
print("polar_img.shape: ", polar_img.shape)
print("cart_img.shape: ", cart_img.shape)

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
        step_angle=step_angle,
    )

    # Convert the polar image to a cartesian image
    cart_img = utils.polar_to_cart(polar_img)

    # detect obstacles
    detection_counter += 1
    if detection_counter == detection_every:
        obstacles = utils.object_detection(polar_img)

        # draw obstacles
        # obs_img = cv2.fillPoly(np.zeros_like(polar_img), [obs.points for obs in obstacles], 255)
        obs_img = np.zeros((size[0], size[1], 3), dtype=np.uint8)
        for obs in obstacles:
            cv2.fillPoly(obs_img, [obs.points], np.random.randint(0, 255, 3).tolist())

        cv2.imshow("Obstacles", obs_img)
        detection_counter = 0

    cv2.imshow("Polar", polar_img)
    cv2.imshow("Cart", cart_img)
    cv2.waitKey(1)
