"""
Utils for the octogon surfacing mission
"""

import cv2
import numpy as np
import logging

import time

logging.basicConfig(level=logging.INFO)


def contours_get_octogon_center(frame, memory_edges):
    """
    Returns the center of the octogon in the frame
    using a contours detection approach
    """

    # convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # apply Canny edge detection
    edges = cv2.Canny(image=gray, threshold1=0, threshold2=200)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)

    # combine with memory edges
    memory_edges = cv2.addWeighted(memory_edges, 0.85, edges, 0.15, 0)
    _, edges = cv2.threshold(memory_edges, 60, 255, cv2.THRESH_BINARY)

    # cv2.imshow("memory_edges", memory_edges)
    # cv2.imshow("edges", edges)

    # find contours, convex hull, and approx poly
    contours, hierarchy = cv2.findContours(
        edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS
    )
    contours = [cv2.convexHull(contour) for contour in contours]
    contours = [contour for contour in contours if cv2.contourArea(contour) > 1000]

    if contours is None or len(contours) == 0:
        return (None, None), memory_edges

    # find the largest contour
    largest_contour = max(contours, key=cv2.contourArea)

    # draw contours
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
    cv2.drawContours(frame, [largest_contour], -1, (0, 255, 255), 2)

    # find the center of the contour
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        x_center = int(M["m10"] / M["m00"])
        y_center = int(M["m01"] / M["m00"])
        cv2.circle(frame, (x_center, y_center), 5, (0, 0, 255), -1)
        return (x_center, y_center), memory_edges
    else:
        return (None, None), memory_edges

def get_error(center_x, center_y, shape):
    """Returns the error in x and y, normalized to the frame size."""
    x_error = (center_x - shape[1] / 2) / (shape[1] / 2)
    y_error = (center_y - shape[0] / 2) / (shape[0] / 2)
    return x_error, y_error

if __name__ == "__main__":
    cap = cv2.VideoCapture("octogon.mp4")
    memory_edges = np.zeros((640, 480), np.uint8)
    memory_error = []
    Done = False

    while not Done:
        start = time.time()

        ret, img = cap.read()
        img = cv2.resize(img, (480, 640))

        if not ret:
            break

        (center_x, center_y), memory_edges = contours_get_octogon_center(img, memory_edges)

        if center_x is None or center_y is None:
            continue
        
        x_error, y_error = get_error(center_x, center_y, img.shape)
        
        # append the error to the memory
        memory_error.append((x_error, y_error))
        # if the memory is full, pop the oldest error
        if len(memory_error) > 60:
            memory_error.pop(0)

        # calculate the average error
        norm_error = np.linalg.norm(memory_error, axis=1)
        norm = np.mean(norm_error)
        if norm < 0.05 and len(memory_error) == 60:
            Done = True
            cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), -1)

        logging.info("norm: {}".format(norm))

        cv2.imshow("img", img)
        # cv2.imshow("memory_edges", memory_edges)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        
        end = time.time()
        logging.info("FPS: {}".format(1 / (end - start)))

    if Done:
        logging.warning("Mission Complete!")
    else:
        logging.warning("Mission Failed...")