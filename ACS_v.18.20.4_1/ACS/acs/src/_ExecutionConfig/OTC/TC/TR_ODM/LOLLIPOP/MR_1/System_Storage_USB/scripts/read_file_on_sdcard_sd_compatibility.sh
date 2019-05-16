#! /bin/bash
# Test case: Storage.USB_SDCard_Read.file.on.sdcard.with.adb.command
# Area: System_Storage/USB
# Test type: Functional Positive
# ET Project: OTC Android Testing
# Author: Diana Dumitrescu

# Prerequisites:
# - have an SDcard inserted in DUT
# Warning! If the prerequisited aren't met the verification will fail the test.
if [ -z "$1" -o -z "$2" -o -z "$3" -o -z "$4" ]
then
        echo "Usage: ./read_file_on_sdcard_sd_compatibility.sh <mounted_folder_1> <fstype1> <mounted_folder2> <fstype2>"
        exit 1
fi

# Verification
out1=$(./scripts/mountstats.sh $1 $2)
out2=$(./scripts/mountstats.sh $3 $4)
if [ $out1 != "PASS" -o $out2 != "PASS" ]
then
        echo "This test requires SDcard inserted into DUT! The SDcard is not inserted or mounted!"
        echo "FAIL"
        exit 1
fi

# Create a file on SDcard, insert somenthing and the read the file from SDcard
adb shell touch /storage/sdcard1/test.txt

# Prerequisites verification
x=$(adb shell 'echo "hello world" > /storage/sdcard1/test.txt')
if [ -n "$x" ]
then
        echo "FAIL"
        exit 1
fi

a=$(adb shell cat /storage/sdcard1/test.txt | tr -d '\r' )

# Compare to see if the content of the created file is as planned
# and if the file can be read
if [ -z "$a" ]
then
        echo "FAIL"
        exit 1
else
        if [ "$a" = "hello world" ]
        then
                echo "PASS"
        else
                echo "FAIL"
                exit 1
        fi
fi

# Cleanup after the execution
adb shell rm /storage/sdcard1/test.txt

exit 0
