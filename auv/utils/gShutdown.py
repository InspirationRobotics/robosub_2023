"""
Handling kill switch on Jetson for the sub

"Turns on" the GPIO pin that when turned on shuts the computer down. 
"""

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
    """For exiting the loop"""
    global loop
    try:
        loop = False
        print("\n\nCleanly Exited")
        exit(1)
    except:
        pass

# Exits the loop when Ctrl + C is pressed
signal.signal(signal.SIGINT, onExit)

# While we haven't exited (while loop == True)
while loop:
    current = GPIO.input(killPin)
    # Check if the GPIO input is "1" (if the kill switch is activated)
    if current == 1:
        state = True
        startTime = time.time()

    # Disarm and shut the sub down
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
    print(GPIO.input(killPin))
    time.sleep(0.1)