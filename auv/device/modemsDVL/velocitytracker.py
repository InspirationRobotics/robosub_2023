# y = integral of sine(heading) wrt time -> velocity with heading accounted for vy = sine(heading) * vy from raw dvl data
# x = integral of cosine(heading) wrt time ->  vx with heading = cosine(heading) * dvl vx
# note average velocity is change of distance over change in time: (x2 - x1)/(time2 -time1)
# output as path on an xy graph
# this should be able to run on computer, just script that takes inputs and outputs graph 
# take input from wasd for forward, left, back, right and +/- for yaw (rotation of heading 0 to 360)

import matplotlib.pyplot as plt
import math  

# start with the sub at (0,0)
vxin = [0]
vyin = [0]
vxforgraph = [0] 
vyforgraph = [0] 
mvmntinput = [0]
headingin = [0]
hforgraph = [0]

mvmt = input("Enter direction of movment (w, a, s, or d): ")
# wasd input temporarilly assign each to have and arbitrary value of 100 mm per second for each input
# for the actual raw dvl velocity data as input have it take in vx, vy so maybe 2 seperate arrays
# positive values is moving right, negative is for moving left
# positive values is moving forward, negative is for moving back 
mvmntinput.append(mvmt)
vxcounter = 0
vycounter = 0
for inputs in mvmntinput:
    if mvmntinput[counter] == 'w':
        vyin.append(1) # move forward mm/s
        vycounter += 1
    elif mvmntinput[counter] == 'a':
        vxin.append(-1) # move left 100 mm/s
        vxcounter += 1
    elif mvmntinput[counter] == 's':
        vyin.append(-1) # move back mm/s
        vycounter += 1
    elif mvmntinput[counter] == 'd':
        vxin.append(1) # move right 100 mm/s
        vxcounter += 1
    else:
        print("Invalid input, check for capitalization")

# variable name for the heading +/- for yaw (rotation of heading 0 to 360)
heading = input("Enter heading direction (+/- 0 to 360): ")
headingin.append(heading)
hcounter = 0
for inputs in headingin:
  degreeheading = headingin[hcounter] * (math.pi/180)
  hforgraph.append(degreeheading)
  hcounter += 1
# heading_fl = float(heading) # was getting errors about this, think it was trying to process the input before there was any input
# convert the degree values to radians so the math function can handle, should be an angle mentioned in terms of radians (pi/2, pi/3/ pi/6, etc)
  
# creating variables that will output the velocity with heading accounted for in meters
# math function returns values in radians 
# iterate through every item in each array
gxcounter = 0
for inputs in vxin
  vxwhead = math.degrees(math.sin(hforgraph[])) * vxin[gcounter]
  vxforgraph.append(vxwhead) 
  vywhead = math.degrees(math.cos(degreeheading)) * vyin[gcounter]
  gxcounter += 1

gycounter = 0
for inputs in vxin
  vywhead = math.degrees(math.cos(degreeheading)) * vyin[gcounter]
  vyforgraph.append(vywhead) 
  gycounter += 1
print("x velocity taking into account the heading:",vxwhead,"| y velocity taking into account the heading:",vywhead) 
# to graph it have the vxwheading and vywheading value pairs into a and x value and y value array to plot  

# plotting the line 2 points 
plt.plot(vxforgraph, vyforgraph)
  
# naming the x axis
plt.xlabel('x-axis in mm') # may want to figure out how to convert in meters
# naming the y axis
plt.ylabel('y-axis in mm')
# giving a title to my graph
plt.title('Velocity Tracking')
# function to show the plot
plt.show()
