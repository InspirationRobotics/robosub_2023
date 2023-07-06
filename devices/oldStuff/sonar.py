from brping import Ping1D
import os

device = "/dev/ttyUSB1"
os.system("sudo chmod 666 " + device)
myPing = Ping1D(device, 115200)

if myPing.initialize() is False:
    print("Failed to initialize Ping!")
    exit(1)