# y = integral of sine(heading) wrt time -> velocity with heading accounted for vy = sine(heading) * vy from raw dvl data
# x = integral of cosine(heading) wrt time ->  vx with heading = cosine(heading) * dvl vx
# note average velocity is change of distance over change in time: (x2 - x1)/(time2 -time1)
# output as path on an xy graph
# this should be able to run on computer, just script that takes inputs and outputs graph 
# take input from wasd for forward, left, back, right and +/- for yaw (rotation of heading 0 to 360)

import math 
import time

# variable names for the raw dvl velocity data 
vx = 0    # positive values is moving right, negative is for moving left
vy = 0    # positive values is moving forward, negative is for moving back 

mvmt = input("Enter direction of movment (w, a, s, or d): ")
# wasd input temporarilly assign each to have and arbitrary value of 100 mm per second for each input
if choice == 'w':
  vy = 1 # move forward mm/s
elif choice == 'a':
  vx = -1 # move left 100 mm/s
elif choice == 's':
  vy = -1 # move back mm/s
elif choice == 'd':
  vx = 1 # move right 100 mm/s
else:
    print("Invalid choice")
  
# variable name for the heading +/- for yaw (rotation of heading 0 to 360)
heading = input("Enter heading direction (+/- 0 to 360): ")
# convert the degree values to radians so the math function can handle, should be an angle mentioned in terms of radians (pi/2, pi/3/ pi/6, etc)
heading = heading*(math.pi/180)
  
# creating variables that will output the velocity with heading accounted for in meters
# math function returns values in radians 
vxwhead = math.degrees(math.sin(heading)) * vx
vywhead = math.degrees(math.cos(heading)) * vy

print("x velocity taking into account the heading:",vxwhead,"| y velocity taking into account the heading:",vywhead) 


