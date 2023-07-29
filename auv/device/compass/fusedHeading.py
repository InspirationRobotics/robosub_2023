import math
import statistics
from time import sleep

import numpy as np
from auv.device.compass.altimu10v5.lis3mdl import LIS3MDL
from auv.device.compass.altimu10v5.lsm6ds33 import LSM6DS33

#lsm6ds33 = LSM6DS33()
lis3mdl = LIS3MDL()

#lsm6ds33.enable()

lis3mdl.enable()
lis3mdl.calibrate(200, True)

headingArr = [0] * 5
count = 0

while True:
    #accel = lsm6ds33.get_accelerometer_g_forces()
    mag = lis3mdl.get_magnetometer_calibrated()
    heading = (math.atan2(mag[1], mag[0]) * 180) / math.pi
    print(heading)
    # E = np.cross(mag, accel)
    # E = E / np.linalg.norm(E)
    # N = np.cross(accel, E)
    # N = N / np.linalg.norm(N)
    # heading = math.atan2(np.dot(E, [1, 0, 0]), np.dot(N, [1, 0, 0]) * 180 / math.pi)
    # if heading < 0:
    #     heading += 360
    # heading = round(heading)
    # if count == 4:
    #     print(statistics.mode(headingArr))
    #     count = 0
    # else:
    #     headingArr[count] = heading
    #     count += 1
    sleep(0.05)
