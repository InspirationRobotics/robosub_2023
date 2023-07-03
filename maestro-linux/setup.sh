#!/bin/bash

sudo apt-get install libusb-1.0-0-dev mono-runtime libmono-system-windows-forms4.0-cil -y

sudo cp 99-pololu.rules /etc/udev/rules.d/99-pololu.rules

echo "Please reboot now"