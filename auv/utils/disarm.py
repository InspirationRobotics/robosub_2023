from ..device.pix_standalone import AUV
from . import statusLed
import os


def disarm():
    auv = AUV()
    auv.arm(False)
    statusLed.red(False)


if __name__ == "__main__":
    disarm()
