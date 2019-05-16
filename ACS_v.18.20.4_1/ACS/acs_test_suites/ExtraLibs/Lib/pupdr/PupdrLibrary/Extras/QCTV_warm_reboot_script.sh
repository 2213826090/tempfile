#!/bin/bash

# usage: ./QCTV_warm_reboot_script.sh <SSN number> <number_of_loops> <path_to_log_file>

DEVICE=$1
LOOP=$2
LOG=$3

cat /dev/null > $LOG

checkForDevice () {
    iter=0;
    adbCheck=`adb devices | grep "$DEVICE" | awk '{print $1}'`
    fastbootCheck=`fastboot devices | grep "$DEVICE" | awk '{print $1}'`
    while [ ! -n adbCheck ] && [ ! -n fastbootCheck ] && [ iter -le 60 ]; do
        sleep 1
        iter=$((iter+1))
    done
    if [[ `adb devices` =~ "$DEVICE" ]];then
        devBridge="adb"
    elif [[ `fastboot devices` =~ "$DEVICE" ]];then
        devBridge="fastboot"
    else
        echo "Device $DEVICE not found. FAIL!!!"
        exit 1
    fi
}

i=1
while [ $i -le $LOOP ]; do
    echo "`date` - Starting iteration $i..."
    echo "`date` - Starting iteration $i..." >> $LOG
    checkForDevice
    if [[ "$devBridge" == "fastboot" ]]; then
        fastboot -s $DEVICE reboot > /dev/null 2>&1
            while [ ! `adb devices` =~ "$DEVICE" ] && [ $iter -le "60" ]; do
                sleep 1
                iter=$((iter+1))
            done
    fi
    echo "WarmBoot LOOP $i started at BENCH time: `date` DUT time: `adb -s $DEVICE shell date`"
    echo "WarmBoot LOOP $i started at BENCH time: `date` DUT time: `adb -s $DEVICE shell date`" >> $LOG
    adb -s $DEVICE reboot
    adb -s $DEVICE wait-for-device
    echo "WarmBoot LOOP $i ended at BENCH time: `date` DUT time: `adb -s $DEVICE shell date`"
    echo "WarmBoot LOOP $i ended at BENCH time: `date` DUT time: `adb -s $DEVICE shell date`"  >> $LOG
    sleep 3
    adb -s $DEVICE root > /dev/null 2>&1
    sleep 3
    iter=0
    while [[ `adb -s $DEVICE shell getprop | grep 'dev.bootcomplete' | wc -l` -ne "1" ]] && [[ "$iter" -le "20" ]]; do
        sleep 1
        iter=$((iter+1))
    done
    if [[ `adb -s $DEVICE shell getprop | grep 'dev.bootcomplete' | wc -l` -ne "1" ]]; then
        echo "warmReboot FAIL"
        echo "warmReboot FAIL" >> $LOG
    else
        echo "warmReboot PASS"
        echo "warmReboot PASS" >> $LOG
    fi
    adb -s $DEVICE shell getprop | grep boot >> $LOG
    adb -s $DEVICE shell dmesg >> $LOG
    adb -s $DEVICE shell "cat /sys/class/power_supply/*/uevent" >> $LOG
    sleep 30
    let i=i+1
done

exit 0