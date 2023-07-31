from math import atan, atan2, degrees, pi, cos, sin
import statistics
import time
import numpy as np
from auv.device.compass.altimu10v5.lis3mdl import LIS3MDL
from auv.device.compass.altimu10v5.lsm6ds33 import LSM6DS33
from collections import deque

lsm6ds33 = LSM6DS33(8)
lis3mdl = LIS3MDL(8)

gyro_cal = [90.355, -129.623, -16.01]
accel_cal = [-1.924527664683261, -0.09843178946432635]

lsm6ds33.enable()
lis3mdl.enable()
#lis3mdl.calibrate(3000, True)
rate = 50 #hz
bufferSize = 10 # number of samples to average
buffer = 0
cnt=0
def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle

while True:
    time.sleep(1/rate)
    ax, ay, az = lsm6ds33.get_accelerometer_g_forces()
    gyro = lsm6ds33.get_gyro_angular_velocity()
    Xm,Ym,Zm = lis3mdl.get_magnetometer_calibrated()
    #print(Xm,Ym,Zm)
    # tilt compensation equations
    stableZ = az + (ax*0.01)
    phi = atan2(ay, stableZ)

    Gz2 = ay * sin(phi) + az * cos(phi)
    theta = atan(-ax/ Gz2)

    By2 = Zm * sin(phi) - Ym * cos(phi)
    Bz2 = Ym * sin(phi) + Zm * cos(phi)
    Bx3 = Xm * cos(theta) + Bz2 * sin(theta)
    
    Psi = vector_2_degrees(Bx3, By2)
    buffer+=Psi
    cnt+=1
    print("Yaw Heading: ", "{:.2f}".format(Psi))
