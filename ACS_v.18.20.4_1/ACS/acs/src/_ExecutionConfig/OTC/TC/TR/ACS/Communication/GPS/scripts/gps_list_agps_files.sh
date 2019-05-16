#!/bin/bash


adb root
adb wait-for-device

#this works only for broadcom chips
adb shell getprop | grep BRCM
if [[ $? != 0 ]]; then
    echo "DUT doesn't seem to have a broadcom gps chip, quit test"
    exit
fi

adb shell "[ -e "/data/gps/lto2.dat" ] && echo "Pass" || echo "Fail""

adb shell "[ -e "/data/gps/ltoStatus.txt" ] && echo "Pass" || echo "Fail""

