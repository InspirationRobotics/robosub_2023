from ..device.pix_standalone import AUV
from . import statusLed
import os

if __name__ == "__main__":
        auv = AUV()
        auv.arm(False)
        statusLed.red(False)
