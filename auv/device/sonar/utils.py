import cv2
import numpy as np


class Obstacle:
    """Stores information about an obstacle detected by the sonar"""

    def __init__(self, points):
        # type: (np.ndarray) -> None
        self.points = points
        self.center = np.mean(points, axis=0)[0]

        self.distance = self.center[0]
        self.angle = self.center[1]

        self.size = np.max(points, axis=0) - np.min(points, axis=0)
        self.area = cv2.contourArea(points)
        self.perimeter = cv2.arcLength(points, True)

    def __repr__(self):
        return "Obstacle({}, {})".format(self.distance, self.angle)

def plot_to_polar_gray(img, angle, points, imsize=(400, 400), step_angle=1):
    """
    plot some points in polar coordinates on an image
    the image seam is at the angle pi (back of the sonar)
    """
    if points is None:
        return

    angle = angle % 400
    num_points = len(points)

    x_factor = imsize[1] / num_points

    for r, value in enumerate(points):
        if value is None:
            continue

        x = int((r + 1) * x_factor)
        y = int(angle)

        # cv2.circle(img, (x, y), thickness, value, -1)
        img[y : y + step_angle, x : int(round(x + x_factor))] = value

    return img


def cart_to_polar(img, imsize=400):
    return cv2.warpPolar(
        img,
        (imsize, imsize),
        (imsize // 2, imsize // 2),
        imsize // 2,
        cv2.WARP_POLAR_LINEAR,
    )


def polar_to_cart(img):
    imsize = img.shape[1]
    return cv2.warpPolar(
        img,
        (imsize, imsize),
        (imsize // 2, imsize // 2),
        imsize // 2,
        cv2.WARP_INVERSE_MAP,
    )


def create_obstacle(min_size=25, max_size=75, min_points=10, max_points=20, imsize=(400, 400)):
    """Create random polygons shapes"""
    num_points = np.random.randint(min_points, max_points)
    object_size = np.random.randint(min_size, max_size)

    position = (np.random.randint(0, imsize[0]), np.random.randint(0, imsize[1]))

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


def object_detection(img, threshold=60):
    # type: (np.array, int) -> list[Obstacle]
    if img is None:
        return []

    if len(img.shape) == 3 and img.shape[2] == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    # remove the first 8% of the points (sub own noise)
    gray[:, : int(gray.shape[1] * 0.08)] = 0

    # blur the image
    gray = cv2.blur(gray, (5, 5), 0)
    cv2.imshow("gray", gray)

    # threshold the image
    _, thresh = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

    # find contours
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    img_contours = np.zeros_like(img)
    cv2.drawContours(img_contours, contours, -1, 255, 2)
    cv2.imshow("contours", img_contours)

    cv2.waitKey(0)

    objects = [Obstacle(contour) for contour in contours]
    return objects