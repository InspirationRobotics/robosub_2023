# by colin
# objective: 
# utilize command to print out heading, collect physically
# log it within config.txt
# turn the robot to the heading, move through the gate

from robot_control import RobotControl
import time
import os
import json

# getting the values from the config file
f = open('config.json')
data = json.load(f)

depth = data['coin_flip'][0]["depth"]
heading = data['coin_flip'][0]["heading"] # in the direction of the gate
# completed loading the values from the json file


rc = RobotControl()
time.sleep(1)
rc.setDepth(depth) #setting depth, robot decends

time.sleep(5) # wait

rc.setHeading(heading) # turning to the heading
rc.forwardDist(7, 2)
# going 12.5 meters forward because in reality auv forwardDist goes 4/5 distance given
#rc.forwardDist(12.5, 2) # moving forward, 10 meters, at 2 power

## running the gate cv code

# need to adjust forwardDist below to make sure we stop at right point for style points
#rc.forwardDist(10, 2) # moving forward, 10 meters, at 2 power

increment = 90

for i in range(1,9): 
    #incrment = increment + increment
    print(heading+increment*i)
    rc.setHeading(heading+increment*i) # turning to the heading

rc.forwardDist(4, 2)
