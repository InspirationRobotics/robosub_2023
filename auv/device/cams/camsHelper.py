import os
#order is forward, down
#onyx = ["platform-3610000.xhci-usb-0:2.1.4:1.0"]
#grey = ["platform-70090000.xusb-usb-0:2.2:1.0","platform-70090000.xusb-usb-0:2.1.1:1.0"]
def findCam(ids):
    bash = os.popen('bash /home/inspiration/auv/auv/device/cams/camsLink.sh').read()
    bash = bash.split("\n")
    result = []
    for id in enumerate(ids):
        result.append("")
        minVal = 100
        for line in bash:
            if "/dev/video" in line:
                line = line.split(" - ")
                if id[1] in line[1]:
                    val = int(line[0][10:])
                    if val < minVal:
                        minVal = val
                        result[id[0]] = line[0]
                else:
                    if(line[1] not in ids):
                        ids.append(line[1])
    for i in reversed(result):
        if i == '':
            result.remove(i)
    return result

#print(findCam(grey))

