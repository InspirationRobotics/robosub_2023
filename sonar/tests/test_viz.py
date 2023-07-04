import numpy as np
import cv2

from sonar.viz import color_image_to_polar, cart_to_polar


def test_draw_polar():
    size = 400
    img = np.zeros((size, size, 3), dtype=np.uint8)

    for angle, points in enumerate([[i for i in range(1, 300)] for _ in range(400)]):
        color_image_to_polar(img, angle, points, imsize=size)
        polar = cart_to_polar(img)
        
        cv2.imshow("cart", img)
        cv2.imshow("polar", polar)

        cv2.waitKey(1)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
