#!/bin/bash

# usage: ./QCTV_cold_boot_script.sh <SSN number> <number_of_loops> <path_to_log_file> <relay_card_tty_name> <power_key_entry_on_relay_card>

DEVICE=$1
LOOP=$2
LOG=$3
SERIAL=$4
PIN=$5

declare -a PIN_ON=(NULL 65 66 67 68 69 6A 6B 6C)
declare -a PIN_OFF=(NULL 6F 70 71 72 73 74 75 76)
ON=${PIN_ON[$PIN]}
OFF=${PIN_OFF[$PIN]}

cat /dev/null > $LOG

i=1
while [ $i -le $LOOP ]; do
    echo "LOOP $i POWER OFF at: `date`" >> $LOG
    echo "LOOP $i POWER OFF at: `date`"
    echo -e "\x${ON}" > $SERIAL;sleep 15; echo -e "\x${OFF}" > $SERIAL
    adb devices | grep "${DEVICE}"
    if [ $? -eq 0 ]
        then
        echo "POWER OFF Failed."
        exit 1
    fi
    sleep 2
    echo -e "\x${ON}" > $SERIAL;sleep 4; echo -e "\x${OFF}" > $SERIAL
    echo "LOOP $i POWER ON at: `date`" >> $LOG
    echo "LOOP $i POWER ON at: `date`"
    adb -s $DEVICE wait-for-device
    sleep 30
    adb -s $DEVICE shell dmesg >> $LOG
    adb -s $DEVICE shell "cat /sys/class/power_supply/*/uevent" >> $LOG
    let i=i+1
done

exit 0