import math
import statistics
from time import sleep

import numpy as np
from .altimu10v5.lis3mdl import LIS3MDL
from .altimu10v5.lsm6ds33 import LSM6DS33

lsm6ds33 = LSM6DS33()
lis3mdl = LIS3MDL()

lsm6ds33.enable()

# If no calib values are provided assumes true north is where sub is pointing when script starts.
# lis3mdl.enable()
# To find calib values, run this script while the sub is pointed towards true north and note down the calib values
# that show up at the start ie. min: [x,x,x] and max: [x,x,x] then replace the above line with
lis3mdl.enable([139, -2497, -2350], [580, -1821, -2069])  # which is min and max respectively

headingArr = [0] * 5
count = 0


while True:
    accel = lsm6ds33.get_accelerometer_g_forces()
    mag = lis3mdl.get_magnetometer_calibrated()
    E = np.cross(mag, accel)
    E = E / np.linalg.norm(E)
    N = np.cross(accel, E)
    N = N / np.linalg.norm(N)
    heading = math.atan2(np.dot(E, [1, 0, 0]), np.dot(N, [1, 0, 0]) * 180 / math.pi)
    if heading < 0:
        heading += 360
    heading = round(heading)
    if count == 4:
        print(statistics.mode(headingArr))
        count = 0
    else:
        headingArr[count] = heading
        count += 1
    sleep(0.05)
