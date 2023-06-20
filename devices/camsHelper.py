import os
#order is forward, down
#onyx = ["Sonix_Technology_Co.__Ltd._H264_USB_Camera_SN0001"]
#grey = ["platform-70090000.xusb-usb-0:2.2:1.0","platform-70090000.xusb-usb-0:2.1.1:1.0"]
def findCam(ids):
    bash = os.popen('./camsLink.sh').read()
    bash = bash.split("\n")
    result = []
    for id in enumerate(ids):
        result.append("")
        minVal = 100
        for line in bash:
            if "/dev/video" in line:
		print(line)
                line = line.split(" - ")
                if id[1] in line[1]:
                    val = int(line[0][10:])
                    if val < minVal:
                        minVal = val
                        result[id[0]] = line[0]
                else:
                    if(line[1] not in ids):
                        ids.append(line[1])
    return result

#print(findCam(grey))

