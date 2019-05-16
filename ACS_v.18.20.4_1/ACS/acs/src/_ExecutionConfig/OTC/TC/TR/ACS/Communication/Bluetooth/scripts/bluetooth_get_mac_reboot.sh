#!/bin/bash

function wait_for_adb {
    i=0
    while [ $i -lt 8 ]; do
        sleep 2
        output=`adb shell settings get secure bluetooth_address`
        if [ "$output" == "$1" ]; then
           break
        fi
        i=`expr $i + 1`
    done
}


adb root
adb shell service call bluetooth_manager 6
echo "Sleep 5 seconds for the bluetooth to be enabled"
sleep 5

MAC_before=`adb shell settings get secure bluetooth_address`
adb reboot
adb wait-for-device
wait_for_adb $MAC_before

MAC_after=`adb shell settings get secure bluetooth_address`
if [ "$MAC_before" == "$MAC_after" ]; then
    echo "Pass"
else
    echo "Fail"
fi
