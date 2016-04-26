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
	TEXT2=`logname`
	STATE0=`xset q | grep 'Caps Lock' | awk '{ n= $4=="on"; print n }'`
	STATE1=`xset q | grep 'Num Lock' | awk '{ n= $8=="on"; print n }'`
	printf "lcdText %8s%8s\r" $TEXT1 $TEXT2 > $SERIALPORT
	sleep 0.02
	printf "lcdMarkerToggle 0 $STATE0\r" > $SERIALPORT
	sleep 0.02
	printf "lcdMarkerToggle 1 $STATE1\r" > $SERIALPORT
	sleep 0.21
done
