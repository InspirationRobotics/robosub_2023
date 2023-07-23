import Jetson.GPIO as GPIO
import sys
import time

bluePin = 37
redPin = 38

GPIO.setmode(GPIO.BOARD)


def red(state):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(redPin, GPIO.OUT)
    if state:
        GPIO.output(redPin, GPIO.HIGH)
        print("Red Light is On!")
    else:
        GPIO.output(redPin, GPIO.LOW)
        print("Red Light is Off!")
    GPIO.cleanup()


def flashRed():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(redPin, GPIO.OUT)
    if time.time() % 2 == 0:
        if GPIO.input(redPin) == 0:
            GPIO.output(redPin, GPIO.HIGH)
        elif GPIO.input(redPin) == 1:
            GPIO.output(redPin, GPIO.LOW)
    GPIO.cleanup()


def blue(state):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(bluePin, GPIO.OUT)
    if state:
        GPIO.output(bluePin, GPIO.HIGH)
        print("Blue Light is On!")
    else:
        GPIO.output(bluePin, GPIO.LOW)
        print("Blue Light is Off!")
    GPIO.cleanup()


if len(sys.argv) > 1:
    for i in range(1, len(sys.argv)):
        if sys.argv[i] == "redOn":
            red(True)
        elif sys.argv[i] == "redOff":
            red(False)
        elif sys.argv[i] == "blueOn":
            blue(True)
        elif sys.argv[i] == "blueOff":
            blue(False)
        else:
            print("Bad argument, ignoring...")

GPIO.cleanup()
