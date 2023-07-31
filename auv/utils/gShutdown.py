import Jetson.GPIO as GPIO
import sys
import time
import os

killPin = 35
global state
state = True
GPIO.setmode(GPIO.BOARD)

GPIO.setup(killPin, GPIO.IN)


def printState(tp):
    global state
    print("Detected")
    GPIO.cleanup()
    state = False
    os.system("sudo poweroff")

GPIO.add_event_detect(killPin, GPIO.FALLING, callback=printState, bouncetime=10)

while state:
    #print(GPIO.input(killPin))
    time.sleep(0.1)