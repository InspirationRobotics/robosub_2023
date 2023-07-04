"""
Script designed to experiment with the Ping360 sonar and collect some data.
"""

import argparse
import logging
import os
import time

from sonar import Ping360

logging.basicConfig(level=logging.INFO)


parser = argparse.ArgumentParser(description='Ping360 data collection script')
parser.add_argument('--device', action="store", required=False, default="/dev/ttyUSB0", type=str, help="Ping360 serial device")
parser.add_argument('--baudrate', action="store", required=False, default=115200, type=int, help="Ping360 serial baudrate")
parser.add_argument('--output', action="store", required=False, default="sonar.txt", type=str, help="Output file name")

# usage example: python ping360.py --device /dev/ttyUSB0 --baudrate 115200 --output sonar.txt

args = parser.parse_args()

# Create a Ping360 object and connect to the Ping360
p = Ping360(
    args.device,
    args.baudrate,
    scan_mode=0,
    angle_range=(0, 399),
    angle_step=5,
)


p.set_transmit_frequency(800)
p.set_sample_period(80) # 25ns units : 400*25ns = 10us
p.set_number_of_samples(200)
p.set_gain_setting(0)

d = p.get_device_data()
print(d)

# make a full scan and save it to a file
logging.info("Starting Ping360 full scan")

while(True):
    try:
        with open(str(time.time()) + ".txt", "wb") as f:
            start = time.time()
            scan = p.full_scan()
            end = time.time()
            logging.info(f"Scan complete in {end-start} seconds")

            f.writelines(scan)
    except KeyboardInterrupt:
        break
