from altimu10v5.lsm6ds33 import LSM6DS33
from altimu10v5.lis3mdl import LIS3MDL
from time import sleep
import numpy as np
import math
import statistics

lsm6ds33 = LSM6DS33()
lis3mdl = LIS3MDL()
lsm6ds33.enable()
lis3mdl.enable()

headingArr = [0]*5
count=0

#Assumes true north is where sub is pointing when script starts. To solve this hardcode mag calib min and mag calib max in altimu10v5/lis3mdl file and comment out calib stuff
#You would have to point sub to true north, calibrate, write down the values the calib function outputs, and then hardcode them ^

while True:
    accel = lsm6ds33.get_accelerometer_g_forces()
    mag = lis3mdl.get_magnetometer_calibrated()
    E = np.cross(mag,accel)
    E = E / np.linalg.norm(E)
    N = np.cross(accel, E)
    N = N / np.linalg.norm(N)
    heading = math.atan2(np.dot(E, [1,0,0]), np.dot(N, [1,0,0]) *  180 / math.pi)
    if(heading<0):
        heading+=360
    heading = round(heading)
    if(count==4):
        print(statistics.mode(headingArr))
        count=0
    else:
        headingArr[count] = heading
        count+=1
    sleep(0.05)

