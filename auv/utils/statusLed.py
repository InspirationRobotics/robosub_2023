"""
To turn the Status LED on the Jetson different colors based on the state: steady red indicates sub is armed,
flashing red indicates voltage of the battery is under 13.5 V, blue is never used
"""

import Jetson.GPIO as GPIO # Import GPIO library specific for Nvidia Jetson
import sys # For system-related functionalities
import time

# Setting the different pins for the red and blue GPIOs
bluePin = 37
redPin = 38

GPIO.setmode(GPIO.BOARD) # Set the GPIO mode to board


def red(state):
    """
    Turn the red LED on
    """
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(redPin, GPIO.OUT) # Set the redPin pin as the output
    if state:
        GPIO.output(redPin, GPIO.HIGH) # Output power to the pin
        print("Red Light is On!")
    else:
        GPIO.output(redPin, GPIO.LOW) # Turn off power (send low power) to the pin
        print("Red Light is Off!")
    GPIO.cleanup()


def flashRed():
    """
    Turn the Red LED on and off at two second intervals
    """
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(redPin, GPIO.OUT) # Set the output pin to the redPin-numbered pin
    # Every time the second meter is even, switch the state (if on turn off, if off turn on)
    if time.time() % 2 == 0:
        if GPIO.input(redPin) == 0:
            GPIO.output(redPin, GPIO.HIGH)
        elif GPIO.input(redPin) == 1:
            GPIO.output(redPin, GPIO.LOW)
    GPIO.cleanup()


def blue(state):
    """Same method as red(), except different pin number"""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(bluePin, GPIO.OUT)
    if state:
        GPIO.output(bluePin, GPIO.HIGH)
        print("Blue Light is On!")
    else:
        GPIO.output(bluePin, GPIO.LOW)
        print("Blue Light is Off!")
    GPIO.cleanup()

# For testing purposes, command line argument(string) can be passed in
# Checking if len of CL argumnet is greater than 1 to make sure an actual CL argument is being passed in
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
