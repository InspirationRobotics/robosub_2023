import math
import statistics
from time import sleep

import numpy as np
from auv.device.compass.altimu10v5.lis3mdl import LIS3MDL
from auv.device.compass.altimu10v5.lsm6ds33 import LSM6DS33

lsm6ds33 = LSM6DS33(1)
lis3mdl = LIS3MDL(1)

lsm6ds33.enable()
lis3mdl.enable()
#lis3mdl.calibrate(3000, True)

while True:
    accel = lsm6ds33.get_accelerometer_g_forces()
    gyro = lsm6ds33.get_gyro_angular_velocity()
    mag = lis3mdl.get_magnetometer_calibrated()
    #heading = (math.atan2(mag[1], mag[0]) * 180) / math.pi
    #print(heading)
    E = np.cross(mag, accel)
    E = E / np.linalg.norm(E)
    N = np.cross(accel, E)
    N = N / np.linalg.norm(N)
    heading = math.atan2(np.dot(E, [1, 0, 0]), np.dot(N, [1, 0, 0]) * 180 / math.pi)
    print(heading)
    sleep(0.05)
