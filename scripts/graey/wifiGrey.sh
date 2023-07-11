#!/bin/bash

sleep 50
while true 
do
    if ping -c 2 -W 1 8.8.8.8 | grep -q '64 bytes'; then
        echo "Connected"
        sleep 5
    else
        echo "Disconnected"
        sleep 2
        sudo nmcli d wifi connect TeamInspiration password Roboticsiscool!
        sleep 10
    fi
done

#sleep 2
#screen -dmS wifi ping -D 8.8.8.8

#no longer needed, was only for buggy wifi issue