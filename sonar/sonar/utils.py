import cv2
import numpy as np


class Obstacle:
    """Stores information about an obstacle detected by the sonar"""

    def __init__(self, points: np.array):
        self.points = points
        self.center = np.mean(points, axis=0)[0]
        
        self.distance = self.center[0]
        self.angle = self.center[1]

        self.size = np.max(points, axis=0) - np.min(points, axis=0)
        self.area = cv2.contourArea(points)
        self.perimeter = cv2.arcLength(points, True)

    def __repr__(self):
        return f"Obstacle({self.center}, {self.size})"


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


def cart_to_polar(img, imsize=400):
    return cv2.warpPolar(
        img,
        (imsize, imsize),
        (imsize // 2, imsize // 2),
        imsize // 2,
        cv2.WARP_POLAR_LINEAR,
    )


def polar_to_cart(img, imsize=400):
    return cv2.warpPolar(
        img,
        (imsize, imsize),
        (imsize // 2, imsize // 2),
        imsize // 2,
        cv2.WARP_INVERSE_MAP,
    )


def create_obstacle(min_size=25, max_size=75, min_points=10, max_points=20, imsize=400):
    """Create random polygons shapes"""
    num_points = np.random.randint(min_points, max_points)
    object_size = np.random.randint(min_size, max_size)
    position = np.random.randint(0, imsize, 2)

    # create random shape
    shape = np.random.rand(num_points, 2) * object_size

    # center and place shape
    shape = shape - np.mean(shape, axis=0) + position

    # make sure shape is convex
    shape = cv2.convexHull(shape.astype(np.int32))
    obstacle = Obstacle(shape)
    return obstacle


def render_obstacles(img, obstacles):
    """
    Render an obstacle on the image
    img is considered to be in polar coordinates (angle, radius)
    obstacle is considered to be in cartesian coordinates (x, y)
    """

    obstacles_points = [obstacle.points for obstacle in obstacles]

    # first render the obstacle on a separate image
    obstacle_img_cart = np.zeros(img.shape, dtype=np.uint8)
    cv2.fillPoly(obstacle_img_cart, obstacles_points, color=255)
    obstacle_img = cart_to_polar(obstacle_img_cart)

    # get the visible part of the obstacle by casting a ray
    # from the left of the image (radius = 0, with variable angle)
    # to the right of the image (radius = size, with variable angle)

    # assuming image is square
    size = img.shape[0]

    for angle in range(size):
        # cast the ray from the left of the image to the right
        for radius in range(size):
            if obstacle_img[angle, radius].any() > 0:
                cv2.circle(img, (radius, angle), 4, 255, -1)
                break

    img = cv2.blur(img, (3, 7), 0)
    return img


def object_detection(img: np.array):
    if img is None:
        return []

    if len(img.shape) == 3 and img.shape[2] == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img

    # apply thresholding
    ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)

    # find contours
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    # img_contours = np.zeros(img.shape, dtype=np.uint8)
    # cv2.drawContours(img_contours, contours, -1, 255, 3)
    # cv2.imshow("contours", img_contours)

    objects = [Obstacle(contour) for contour in contours]
    return objects