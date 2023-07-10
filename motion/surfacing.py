"""
Main script for surfacing mission
"""
import logging

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from robot_control import RobotControl

logging.basicConfig(level=logging.DEBUG)


def get_octogon_center(frame, memory_edges):
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


# initialize ROS subscribers and publishers
rc = RobotControl()
br = CvBridge()
pubDown = rospy.Publisher("/auv/camera/videoUSBOutput1", Image, queue_size=10)
global downVideo
downVideo = None
rospy.init_node("SurfacingCV", anonymous=True)
rospy.Rate(30)


def callbackDown(msg):
    global downVideo
    try:
        downVideo = br.imgmsg_to_cv2(msg)
    except Exception as e:
        print("Down Output Error, make sure running in Python2")
        print(e)


rospy.Subscriber("/auv/camera/videoUSBRaw1", Image, callbackDown)

# a small loop to wait for video input
frame = None
while frame is None:
    frame = downVideo
    logging.warning("Waiting for videoRaw1...")
    time.sleep(1.0)

# initialize some variables
memory_edges = np.zeros_like(frame)
memory_error = []
memory_length = 30
sensivity = 2.0
Done = False

# start by going up to enlarge the range of the camera
previous_depth = rc.previous_depth
rc.setDepth(0.75)
time.sleep(5.0)  # wait for the sub to stabilize

while not Done:
    try:
        frame = downVideo

        # get the center of the octogon
        (center_x, center_y), memory_edges = get_octogon_center(frame, memory_edges)

        # if no center found, skip
        if center_x is None or center_y is None:
            continue

        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        # calculate the error
        x_error, y_error = get_error(center_x, center_y, frame.shape)

        # strafe left/right and up/down
        # small adjustments to make sure the sub is centered
        rc.movement(
            lateral=x_error * sensivity,
            forward=y_error * sensivity,
        )

        # append the error to the memory
        memory_error.append((x_error, y_error))

        # if the memory is full, pop the oldest error
        if len(memory_error) > memory_length:
            memory_error.pop(0)

        # calculate the average error
        norm_error = np.linalg.norm(memory_error, axis=1)
        norm = np.mean(norm_error)

        # if the average error is small and stable, we can surface
        if norm < 0.05 and len(memory_error) == memory_length:
            Done = True

        logging.info("norm: {}".format(norm))

    except KeyboardInterrupt:
        break

    except Exception as e:
        logging.error(e)
        pass

    finally:
        # publish the frame for debugging
        pubDown.publish(br.cv2_to_imgmsg(frame))

if Done:
    logging.warning("Surfacing mission succeeded, going to the surface")
    os.system("python3 /home/inspiration/auv/devices/disarm.py")
else:
    logging.warning("Surfacing mission failed, going back to previous depth")
    rc.setDepth(previous_depth)
