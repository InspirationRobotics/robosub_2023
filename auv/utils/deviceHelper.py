import os
import sys
import platform
from dotenv import dotenv_values

if("nx" in platform.node()):
    onyx=True
    variables = dotenv_values("/home/inspiration/auv/config/onyx.env")
else:
    onyx=False
    variables = dotenv_values("/home/inspiration/auv/config/graey.env")

def findFromId(ids):
    bash = os.popen('bash /home/inspiration/auv/auv/utils/usbLink.sh').read()
    bashSplit = bash.split("\n")
    result = []
    for id in enumerate(ids):
        result.append("")
        for line in bashSplit:
            if "/dev/tty" in line:
                line = line.split(" - ")
                if id[1] in line[1]:
                        result[id[0]] = line[0]
    for i in reversed(result):
        if i == '':
            result.remove(i)
    if len(result)==0:
        print(bash)
        return ["Device not found, above is list of all available devices"]
    return result[0]

def findCam(ids):
    bash = os.popen('bash /home/inspiration/auv/auv/utils/usbLink.sh').read()
    bash = bash.split("\n")
    result = []
    for id in enumerate(ids):
        result.append("")
        minVal = 100
        for line in bash:
            if "/dev/video" in line:
                line = line.split(" - ")
                if id[1] == line[1]:
                    val = int(line[0][10:])
                    if val < minVal:
                        minVal = val
                        result[id[0]] = line[0]
    for i in reversed(result):
        if i == '':
            result.remove(i)
    return result

def dataFromConfig(name):
    data = None
    usbID = None
    if(name=="forwardOak"):
        data = variables.get("forwardOak_camera_mxid")
    elif(name=="bottomOak"):
        data = variables.get("bottomOak_camera_mxid")
    elif(name=="poeOak"):
        data = variables.get("poeOak_camera_mxid")
    elif(name=="forwardUSB"):
        data = variables.get("forward_camera_port")
    elif(name=="bottomUSB"):
        data = variables.get("bottom_camera_port")
    elif(name=="pixhawk"):
        usbID = variables.get("pixhawk_port")
    elif(name=="modem"):
        usbID = variables.get("modem_port")
    elif(name=="dvl"):
        usbID = variables.get("dvl_port")
    elif(name=="polulu"):
        usbID = variables.get("polulu_port")
    elif(name=="teensy"):
        usbID = variables.get("teensy_port")
    else:
        data = variables.get(name)
        if(data==None):
            raise Exception("Invalid Name")
    if(data!=None):
        return data
    if(usbID==None):
        return None #id is not on sub so leave it
    return findFromId([usbID])
    

if __name__ == '__main__':
    if len(sys.argv)>1:
        print(dataFromConfig(sys.argv[1]))
    else:
        print(dataFromConfig("pixhawk"))