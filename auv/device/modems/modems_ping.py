from . import modems_api
from ...utils.deviceHelper import variables

import time

modem = modems_api.Modem(auto_start=False)
config = variables
addr = 111
if (variables.get("sub", "graey") == "onyx"):
    addr = 222

count = 0
maxPings = 7 #max pings before timeout
data = None


while (count < maxPings and data == None):
    temp = modem.ping_status(111)
    if "#R" in temp: 
        data = temp
    modem.led.on_send_msg()
    time.sleep(0.5)
    count += 1

print("Ping Status: ", data)
#modem.stop()