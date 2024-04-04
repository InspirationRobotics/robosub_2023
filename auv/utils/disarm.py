"""
To disarm the sub by setting the state of arm to be False
Turns the red status LED off to indicate sub is not armed
"""

from ..device.pix_standalone import AUV
from . import statusLed
import os


def disarm():
    auv = AUV()
    auv.arm(False)
    statusLed.red(False)


if __name__ == "__main__":
    disarm()
