import cv2
import numpy as np

from sonar import utils


def test_draw_polar():
    size = 400
    img = np.zeros((size, size, 3), dtype=np.uint8)

    for angle, points in enumerate([[i for i in range(1, 300)] for _ in range(400)]):
        utils.color_image_to_polar(img, angle, points, imsize=size)
        cartesian = utils.polar_to_cart(img)

    cv2.imshow("cart", cartesian)
    cv2.imshow("polar", img)
    cv2.waitKey(5000)


def test_dummy_obstacle():
    size = 400
    img = np.zeros((size, size, 1), dtype=np.uint8)
    obstacles = [utils.create_obstacle(imsize=size) for _ in range(10)]
    img = utils.render_obstacles(img, obstacles)

    cartesian = utils.polar_to_cart(img)
    cartesian_filled = cv2.fillPoly(cartesian.copy(), [obs.points for obs in obstacles], color=255)
    polar_filled = utils.cart_to_polar(cartesian_filled)

    cv2.imshow("obstacles_polar", img)
    cv2.imshow("obstacles_cart", cartesian)
    cv2.imshow("obstacles_cart_filled", cartesian_filled)
    cv2.imshow("obstacles_polar_filled", polar_filled)
    cv2.waitKey(5000)


def test_obstacles_detection():
    size=400
    img = np.zeros((size, size, 1), dtype=np.uint8)
    obstacles = [utils.create_obstacle(imsize=size) for _ in range(7)]
    img = utils.render_obstacles(img, obstacles)

    obstacles_detected = utils.object_detection(img)

    img_detected = np.zeros_like(img)
    img_detected = cv2.fillPoly(img_detected, [obs.points for obs in obstacles_detected], color=255)

    cv2.imshow("obstacles", img)
    cv2.imshow("obstacles_detected", img_detected)
    cv2.waitKey(0)
