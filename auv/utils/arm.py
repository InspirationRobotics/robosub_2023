"""
Arm the sub by setting status of arm to True
Turn the red status LED on to indicate the sub is armed
"""

from ..device.pix_standalone import AUV
from . import statusLed
import os


def arm():
    auv = AUV()
    auv.arm(True)
    statusLed.red(True)


if __name__ == "__main__":
    arm()
