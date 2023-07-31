import cv2
import numpy as np

from auv.device.sonar import utils


def test_draw_polar():
    size = (400, 400)
    img = np.zeros((size[0], size[1], 1), dtype=np.uint8)

    step_angle = 2
    for angle, points in enumerate([[i for i in range(0, 255)] for _ in range(400 // step_angle)]):
        angle *= step_angle
        img = utils.plot_to_polar_gray(img, angle, points, imsize=size, step_angle=step_angle)
        cartesian = utils.polar_to_cart(img)

    cv2.imshow("cart", cartesian)
    cv2.imshow("polar", img)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()


def test_dummy_obstacle():
    size = (400, 400)
    img = np.zeros((size[0], size[1], 1), dtype=np.uint8)
    obstacles = [utils.create_obstacle(imsize=size) for _ in range(10)]
    img = utils.render_obstacles(img, obstacles)

    cartesian = utils.polar_to_cart(img)
    cartesian_filled = cv2.fillPoly(cartesian.copy(), [obs.points for obs in obstacles], color=255)
    polar_filled = utils.cart_to_polar(cartesian_filled)

    cv2.imshow("obstacles_polar", img)
    cv2.imshow("obstacles_cart", cartesian)
    cv2.imshow("obstacles_cart_filled", cartesian_filled)
    cv2.imshow("obstacles_polar_filled", polar_filled)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()


def test_obstacles_detection():
    size = (400, 400)
    img = np.zeros((size[0], size[1], 1), dtype=np.uint8)
    obstacles = [utils.create_obstacle(imsize=size) for _ in range(7)]
    img = utils.render_obstacles(img, obstacles)

    obstacles_detected = utils.object_detection(img)

    img_detected = np.zeros_like(img)
    img_detected = cv2.fillPoly(img_detected, [obs.points for obs in obstacles_detected], color=255)

    cv2.imshow("obstacles", img)
    cv2.imshow("obstacles_detected", img_detected)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()
