import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

blue = 37
red = 38

GPIO.setup(red, GPIO.OUT)
GPIO.setup(blue, GPIO.OUT)

GPIO.output(blue, GPIO.HIGH)
print("Blue Light is On!")


GPIO.cleanup()