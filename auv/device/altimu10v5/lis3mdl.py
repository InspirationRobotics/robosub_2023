# -*- coding: utf-8 -*-

"""Python library module for LIS3MDL magnetometer.
This module for the Raspberry Pi computer helps interface the LIS3MDL
magnetometer.The library makes it easy to read the raw magnetometer
through I²C interface.

The datasheet for the LSM6DS33 is available at
[https://www.pololu.com/file/download/LIS3MDL.pdf?file_id=0J1089]
"""

from .i2c import I2C
from .constants import *
import time


class LIS3MDL(I2C):
    """Set up and access LIS3MDL magnetometer."""

    # Output registers used by the magnetometer
    magnetometer_registers = [
        LIS3MDL_OUT_X_L,  # low byte of X value
        LIS3MDL_OUT_X_H,  # high byte of X value
        LIS3MDL_OUT_Y_L,  # low byte of Y value
        LIS3MDL_OUT_Y_H,  # high byte of Y value
        LIS3MDL_OUT_Z_L,  # low byte of Z value
        LIS3MDL_OUT_Z_H,  # high byte of Z value
    ]

    def __init__(self, bus_id=8):
        """Set up I2C connection and initialize some flags and values."""

        super(LIS3MDL, self).__init__(bus_id)
        self.is_magnetometer_enabled = False
        self.is_mag_calibrated = False
        self.mag_cal_min = [32767, 32767, 32767]
        self.mag_cal_max = [-32768, -32768, -32768]

    def __del__(self):
        """Clean up."""
        try:
            # Power down magnetometer
            self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x03)
            super(LIS3MDL, self).__del__()
        except:
            pass

    def enable(self, minCal=None, maxCal=None):
        """Enable and set up the the magnetometer and determine
        whether to auto increment registers during I2C read operations.
        """

        # Disable magnetometer and temperature sensor first
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, 0x00)
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x03)

        # Enable device in continuous conversion mode
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG3, 0x00)

        # Initial value for CTRL_REG1
        ctrl_reg1 = 0x00

        # Ultra-high-performance mode for X and Y
        # Output data rate 10Hz
        # binary value -> 01110000b, hex value -> 0x70
        ctrl_reg1 += 0x70

        # +/- 4 gauss full scale
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG2, 0x00)

        # Ultra-high-performance mode for Z
        # binary value -> 00001100b, hex value -> 0x0c
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG4, 0x0C)

        self.is_magnetometer_enabled = True

        # Write calculated value to the CTRL_REG1 register
        self.write_register(LIS3MDL_ADDR, LIS3MDL_CTRL_REG1, ctrl_reg1)
        time.sleep(0.1)
        if minCal == None or maxCal == None:
            self.calibrate()
        else:
            self.mag_cal_min = minCal
            self.mag_cal_max = maxCal
            self.is_mag_calibrated = True
            print(f"min: {str(self.mag_cal_min)}")
            print(f"max: {str(self.mag_cal_max)}")

    def calibrate(self, iterations=1000):
        """Calibrate the mags raw values."""
        print("Calibrating Magnetometer...")

        for i in range(iterations):
            mag_raw = self.get_magnetometer_raw()

            self.mag_cal_min[0] = min(self.mag_cal_min[0], mag_raw[0])
            self.mag_cal_min[1] = min(self.mag_cal_min[1], mag_raw[1])
            self.mag_cal_min[2] = min(self.mag_cal_min[2], mag_raw[2])

            self.mag_cal_max[0] = max(self.mag_cal_max[0], mag_raw[0])
            self.mag_cal_max[1] = max(self.mag_cal_max[1], mag_raw[1])
            self.mag_cal_max[2] = max(self.mag_cal_max[2], mag_raw[2])

            time.sleep(0.01)
        print("Calibration Done")
        print(f"min: {str(self.mag_cal_min)}")
        print(f"max: {str(self.mag_cal_max)}")
        self.is_mag_calibrated = True

    def get_magnetometer_raw(self):
        """Return 3D vector of raw magnetometer data."""
        # Check if magnetometer has been enabled
        if not self.is_magnetometer_enabled:
            raise (Exception("Magnetometer is not enabled"))

        return self.read_3d_sensor(LIS3MDL_ADDR, self.magnetometer_registers)

    def get_magnetometer_calibrated(self):
        if not self.is_mag_calibrated:
            raise (Exception("Magnetometer is not calibrated"))
        magCalib = self.get_magnetometer_raw()
        magCalib[0] -= (self.mag_cal_min[0] + self.mag_cal_max[0]) / 2
        magCalib[1] -= (self.mag_cal_min[1] + self.mag_cal_max[1]) / 2
        magCalib[2] -= (self.mag_cal_min[2] + self.mag_cal_max[2]) / 2
        return magCalib
