#!/bin/bash

# To read off the information for each device connected by USB to the sub

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        # Extract the system path(path in /sys)
        syspath="${sysdevpath%/dev}"

        # Get the device name using udevadm
        devname="$(udevadm info -q name -p $syspath)"

        # Skip device names starting with "bus/"
        [[ "$devname" == "bus/"* ]] && exit

        # Export device properties, evaluate them (concatenates arguments into single command, executes in current shell environment)
        eval "$(udevadm info -q property --export -p $syspath)"

        # Skip if the ID path or ID serial number is empty
        [[ -z "$ID_PATH" ]] && exit
	    [[ -z "$ID_SERIAL" ]] && exit

        # Print device information
        echo "/dev/$devname - $ID_PATH - $ID_SERIAL"
    )
done
