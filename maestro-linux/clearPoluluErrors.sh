#!/bin/bash

DEVICE=$1

sudo chmod 666 $DEVICE

byte() {
  printf "\\x$(printf "%x" $1)"
}
while true
do
  stty raw -F $DEVICE
  {
    byte 0xA1
  } > $DEVICE
  echo "Cleared Errors"
  sleep 10
done