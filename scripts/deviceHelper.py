import os
import sys
import platform

def findDevice(ids):
    bash = os.popen('bash /home/inspiration/auv/devices/cams/camsLink.sh').read()
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
    return result

if len(sys.argv)>1:
    print(findDevice([sys.argv[1]])[0])
else:
    if("nx" in platform.node()):
        ID = ["platform-3610000.xhci-usb-0:2.3.3:1.0"]
    else:
        ID = None #need to check on graey what id is
    print(findDevice(ID)[0])