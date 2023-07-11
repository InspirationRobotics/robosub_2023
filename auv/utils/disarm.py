from .pix_standalone import AUV
import os

if __name__ == "__main__":
        auv = AUV()
        auv.arm(False)

os.system("/usr/bin/python3 /home/inspiration/auv/devices/statusLed.py redOff")

