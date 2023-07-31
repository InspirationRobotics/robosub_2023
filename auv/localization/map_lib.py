# everything here is radians [0, 2π], every angle > 2π is `angle mod 2π`

from math import *

PI = 3.14159265358979


def init_mapgrid(orientation, center_pose, size, mapgrid):
    mapgrid["orientation"] = orientation
    mapgrid["center"] = center_pose

    data = []
    for i in range(0, size[0]):
        data.append([])

    for i in data:
        for j in range(0, size[1]):
            i.append([])

    mapgrid["data"] = data


def norm_ang(a):
    if a > 2 * PI:
        return a % 360

    if a < 0:
        return (2 * 3.14159) + (a % -(2 * PI))

    return a


def ang_pts(p1, p2):
    if p2[0] - p1[0] == 0:
        return PI / 2

    angle = atan(abs(p2[1] - p1[1]) / abs(p2[0] - p1[0]))
    if p1[0] > p2[0]:
        angle = PI - angle
    if p1[1] > p2[1]:
        angle = 2 * PI - angle

    return angle


def rotate(p, origin, r):
    magnitude = sqrt(pow(abs(p[1] - origin[1]), 2) + pow(abs(p[0] - origin[0]), 2))
    angle = angle_between_points(origin, p)
    r_angle = normalize_angle(r + angle)

    return [magnitude * cos(r_angle), magnitude * sin(r_angle)]


def map_hdg(hdg, map_orientation):
    return normalize_angle(hdg - map_orientation)


def map_pose(dp, map_orientation):
    pass
