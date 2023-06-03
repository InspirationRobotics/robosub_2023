channelsTest  = [1500]*18

def movement(**array):
        channels = channelsTest
        if("throttle" in array):
            channels[2] = ((array["throttle"]*80) + 1500)
        if("forward" in array):
            channels[4] = ((array["forward"]*80) + 1500)
        if("lateral" in array):
            channels[5] = ((array["lateral"]*80) + 1500)
        if("yaw" in array):
            channels[3] = ((array["yaw"]*80) + 1500)
        if("pitch" in array):
            channels[6] = ((array["pitch"]*80) + 1500)
        if("roll" in array):
            channels[7] = ((array["roll"]*80) + 1500)
        print(channels)

movement(forward=2)
