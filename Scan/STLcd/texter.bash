#!/usr/bin/env bash
# STLcd
# Jacob Alexander 2015
# dzonder 2016

if [ $# -eq 0 ]; then
  echo "You must specify your virtual serialport. (/dev/ttyACM0 on linux, /dev/cu.usbmodemXXXX on OSX)"
  echo "  ex: $0 /dev/ttyACM0"
  exit 1
fi

SERIALPORT=$1

printf "\r" > $SERIALPORT

sleep 0.02

while true; do
	TEXT1=`date "+%H:%M:%S"`
	TEXT2=`who am i | awk '{print $1}'`
	printf "lcdText %8s%8s\r" $TEXT1 $TEXT2 > $SERIALPORT
	sleep 0.25
done
