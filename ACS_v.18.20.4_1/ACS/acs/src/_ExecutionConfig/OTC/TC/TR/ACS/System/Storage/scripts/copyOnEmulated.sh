#! /bin/bash
# Test case: Storage.USB_ADB_Copy10000TimesOnEmulated
# Area: System_Storage/USB
# Test type: Reliability - Iterative
# ET Project: OTC Android Testing
# Author: Diana Dumitrescu (dmdumitx)

# parameter verification
if [ -z $1 -a -z $2 ]
then
        echo "Usage: ./copyOnEmulated.sh <nrOfIterations> <fileziseInKB>"
        exit 1
fi

# setup
dd if=/dev/zero of=fileA count=$2 bs=1024 > /dev/null

adb push fileA /sdcard/
adb shell cp /sdcard/fileA /sdcard/fileB

# iteration for the copy process
for i in {1..$1}
do
        adb shell cp /sdcard/fileB /sdcard/fileC
        adb shell cp /sdcard/fileC /sdcard/fileB
done

# verification
cmd1=$(adb shell cmp -l /sdcard/fileB /sdcard/fileC)
cmd2=$(adb shell cmp -l /sdcard/fileA /sdcard/fileB)
if [ -z $cmd1 -a -z $cmd2 ]
then
        echo "PASS"
else
        echo "FAIL"
fi

# cleanup
rm fileA
adb shell rm /sdcard/fileA
adb shell rm /sdcard/fileB
adb shell rm /sdcard/fileC
