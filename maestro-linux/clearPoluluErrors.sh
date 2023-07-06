#!/bin/bash

DEVICE=$1

byte() {
  printf "\\x$(printf "%x" $1)"
}
stty raw -F $DEVICE
{
  byte 0xA1
} > $DEVICE
