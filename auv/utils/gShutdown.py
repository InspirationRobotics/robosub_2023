import Jetson.GPIO as GPIO
import sys
import time
import os
from auv.utils.disarm import disarm
from auv.utils.deviceHelper import variables
import signal


if (variables.get("sub", "onyx") == "onyx"):
    killPin = 19
else:
    killPin=35
global state
global loop
startTime = 0
state = True
loop = True

GPIO.setmode(GPIO.BOARD)

GPIO.setup(killPin, GPIO.IN)


def onExit(signum, frame):
    global loop
    try:
        loop = False
        print("\n\nCleanly Exited")
        exit(1)
    except:
        pass

signal.signal(signal.SIGINT, onExit)

while loop:
    current = GPIO.input(killPin)
    if current == 1:
        state = True
        startTime = time.time()

    if current == 0 and state == True and startTime!=0:
        if time.time() - startTime > 1:
            print("Disarming")
            try:
                disarm()
            except Exception as e:
                print(f"[ERROR]: {e}\ndisarming failed, maybe mavros isn't running ?")
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(killPin, GPIO.IN)
            state = False
    #print(GPIO.input(killPin))
    time.sleep(0.1)