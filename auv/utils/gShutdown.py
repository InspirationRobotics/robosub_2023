import Jetson.GPIO as GPIO
import sys
import time
import os
from .disarm import disarm

killPin = 35 #19 on onyx
global state
startTime = 0
state = True

GPIO.setmode(GPIO.BOARD)

GPIO.setup(killPin, GPIO.IN)

while True:
    current = GPIO.input(killPin)
    if current == 1:
        state = True
        startTime = time.time()

    if current == 0 and state == False:
        if time.time() - startTime > 1:
            print("Disarming")
            disarm()
            state = False
    print(GPIO.input(killPin))
    time.sleep(0.1)