# y = integral of sine(heading) wrt time -> velocity with heading accounted for vy = sine(heading) * vy from raw dvl data
# x = integral of cosine(heading) wrt time ->  vx with heading = cosine(heading) * dvl vx
# note average velocity is change of distance over change in time: (x2 - x1)/(time2 -time1)
# output as path on an xy graph
# this should be able to run on computer, just script that takes inputs and outputs graph 
# take input from wasd for forward, left, back, right and +/- for yaw (rotation of heading 0 to 360)

import matplotlib.pyplot as plt
import math  

# start with the sub at (0,0)
mvmntxinput = []
mvmntyinput = []
vxin = []
vyin = []
headingin = []
hforgraphcalc = []
sumx = []
sumy = []
times = []
x = []
y = []
#xgraph = []
#ygraph = []

count = 0
while (count < 5):
    count = count + 1
    mvmtx, mvmty, strheading, currtime = input("Enter x and y velocity seperated with a space (w, a, s, or d), heading (+/- 0 to 360), and time in seconds: ").split()
# wasd input temporarilly assign each to have and arbitrary value of 10 mm per second for each input
# for the actual raw dvl velocity data as input have it take in vx, vy so 2 seperate arrays
# vxin positive values is moving right, negative is for moving left
# vyin positive values is moving forward, negative is for moving back 
# also will be getting a time stamp when get actual data 
    mvmntxinput.append(mvmtx)
    mvmntyinput.append(mvmty)
    heading = int(strheading)
    headingin.append(heading)
    numtime = int(currtime)
    times.append(numtime)

#convert the input to a number 
vxcounter = 0
vycounter = 0
for inputs in mvmntxinput:
    if mvmntxinput[vxcounter] == 'a':
        vxin.append(-100) # move left 100 mm/s
        print("vxin values:", vxin)
        vxcounter += 1
    elif mvmntxinput[vxcounter] == 'd':
        vxin.append(100) # move right 100 mm/s
        print("vxin values:", vxin)
        vxcounter += 1
    else:
        print("Invalid input, check for capitalization")

for inputs in mvmntyinput:
    if mvmntyinput[vycounter] == 'w':
        vyin.append(100) # move forward 100 mm/s
        print("vyin values:", vyin)
        vycounter += 1
    elif mvmntyinput[vycounter] == 's':
        vyin.append(-100) # move back 100 mm/s
        print("vyin values:", vyin)
        vycounter += 1
    else:
        print("Invalid input, check for capitalization")


hcounter = 0
for inputs in headingin:
  radheading = math.radians(headingin[hcounter])
  hforgraphcalc.append(radheading)
  print("hforgraphcalc values:", hforgraphcalc)
  hcounter += 1
# heading_fl = float(heading) # was getting errors about this, think it was trying to process the input before there was any input
# convert the degree values to radians so the math function can handle, should be an angle mentioned in terms of radians (pi/2, pi/3/ pi/6, etc)
  
# creating variables that will output the velocity with heading accounted for in meters
# math function returns values in radians 
# iterate through every item in each array
gxycounter = 0
gcounter = 1 
for inputs in vxin:
  vxwhead = math.cos(hforgraphcalc[gxycounter]) * vxin[gxycounter]
  #vxwhead = math.degrees(vxwheadrad)
  sumx += vxwhead * (times[gcounter] - times[gxycounter])
  x.append(sumx) 
  vywhead = math.sin(hforgraphcalc[gxycounter]) * vyin[gxycounter]
  #vywhead = math.degrees(vywheadrad)
  sumy += vywhead * (times[gcounter] - times[gxycounter])
  y.append(sumy) 
  print("sumx values:", sumx, "sumy values:", sumy)
  gxycounter += 1
  gcounter += 1 

#gcounter = 1 # does this start at second timestamp
#while gcounter < len(times):
#for inputs in times:
#    x = sumx[gcounter] * (times[gcounter] - times[gcounter - 1])
#    xgraph.append(x)
#    y = sumy[gcounter] * (times[gcounter] - times[gcounter - 1])
#    ygraph.append(y)
#    print(x, y)
#    gcounter += 1

# to graph it have the x and y value pairs into a and x value and y value array to plot  
plt.plot(x, y)
  
# naming the x axis
plt.xlabel('x-axis in mm') # may want to figure out how to convert in meters
# naming the y axis
plt.ylabel('y-axis in mm')
# giving a title to my graph
plt.title('Velocity Tracking')
# function to show the plot
plt.show()
