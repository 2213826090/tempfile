#! /bin/bash
# Test case: Storage.USB_SDCard_CreateFileWith255charsName
# Area: System_Storage/USB
# Test type: Reliability - Iterative
# ET Project: OTC Android Testing
# Author: Diana Dumitrescu (dmdumitx)

# auxiliary function
function check_mounted {
        step0a=$(bash ./scripts/mountstats.sh /mnt/media_rw/sdcard1 vfat)
        step0b=$(bash ./scripts/mountstats.sh /storage/sdcard1 fuse)
        if [ $step0a != "PASS" -o $step0b != "PASS" ]
        then
                echo "FAIL - SDcard is not mounted"
                adb shell am force-stop com.android.settings
                exit 1
        fi
}

# parameter verification
if [ -z $1 ]
then
        echo "Usage: ./createFileSDcard.sh <name>"
        exit 1
fi

# verify if the SDcard is mounted
check_mounted

output2=$(adb shell touch /storage/sdcard1/$1)
if [ $output2 ]
then
        echo "FAIL - Touch command failed"
        exit 1
fi

# checkup
output3=$(adb shell ls /storage/sdcard1 | grep $1)
if [ -z $output3 ]
then
        echo "FAIL - File is not created"
        exit 1
else
        echo "PASS"
fi
