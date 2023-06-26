import Jetson.GPIO as GPIO
import sys

GPIO.setmode(GPIO.BOARD)

bluePin = 37
redPin = 38

def red(state):
    GPIO.setup(redPin, GPIO.OUT)
    if(state):
        GPIO.output(redPin, GPIO.HIGH)
        print("Red Light is On!")
    else:
        GPIO.output(redPin, GPIO.LOW)
        print("Red Light is Off!")

def blue(state):
    GPIO.setup(bluePin, GPIO.OUT)
    if(state):
        GPIO.output(bluePin, GPIO.HIGH)
        print("Blue Light is On!")
    else:
        GPIO.output(bluePin, GPIO.LOW)
        print("Blue Light is Off!")

if len(sys.argv)>1:
    for i in range(1,len(sys.argv)):
        if(sys.argv[i]=='redOn'):
            red(True)
        elif(sys.argv[i]=='redOff'):
            red(False)
        elif(sys.argv[i]=='blueOn'):
            blue(True)
        elif(sys.argv[i]=='blueOff'):
            blue(False)
        else:
            print("Bad argument, ignoring...")

GPIO.cleanup()
