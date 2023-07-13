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

def findFromName(name):
    if(name=="forwardOak"):
        usbID = variables.get("forwardOak_camera_mxid")
    elif(name=="bottomOak"):
        usbID = variables.get("bottomOak_camera_mxid")
    elif(name=="poeOak"):
        usbID = variables.get("poeOak_camera_mxid")
    elif(name=="forwardUSB"):
        usbID = variables.get("forward_camera_port")
    elif(name=="bottomUSB"):
        usbID = variables.get("bottom_camera_port")
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
        raise Exception("Invalid Name")
    findFromId([usbID])
    

if __name__ == '__main__':
    if len(sys.argv)>1:
        print(findFromName([sys.argv[1]]))
    else:
        print(findFromName("pixhawk"))