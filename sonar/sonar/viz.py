import cv2
import numpy as np

def color_image_to_polar(img, angle, points, imsize=400):
    """
    plot some points in cartesian coordinates on an image
    then convert the image to polar
    """
    if points is None:
        return

    angle = angle % 400
    num_points = len(points)

    norm_factor = imsize / num_points
    thickness = int(norm_factor * 0.5 + 1)
    color = [255, 0, (angle / 360) * 255]

    for r, value in enumerate(points):
        if value is None:
            continue
        
        x = int((r + 1) * norm_factor)
        y = int(angle)

        color[1] = value
        cv2.circle(img, (x, y), thickness, color, -1)

def cart_to_polar(img):
    # assume square image 
    size = img.shape[0]
    return cv2.warpPolar(img, (size, size), (size // 2, size // 2), size // 2, cv2.WARP_INVERSE_MAP)
