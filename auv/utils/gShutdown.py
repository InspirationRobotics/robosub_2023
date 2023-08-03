import Jetson.GPIO as GPIO
import sys
import time
import os

killPin = 35
global state
startTime = 0
state = True

GPIO.setmode(GPIO.BOARD)

GPIO.setup(killPin, GPIO.IN)

while state:
    current = GPIO.input(killPin)
    if current == 1:
        startTime = time.time()
    if current == 0:
        if time.time() - startTime > 1:
            print("Shutting down")
            GPIO.cleanup()
            state = False
            break
    print(GPIO.input(killPin))
    time.sleep(0.1)

os.system("sudo poweroff")
exit(1)
