from ..device.pix_standalone import AUV
from . import statusLed
import os


def arm():
    auv = AUV()
    auv.arm(True)
    statusLed.red(True)


if __name__ == "__main__":
    arm()
