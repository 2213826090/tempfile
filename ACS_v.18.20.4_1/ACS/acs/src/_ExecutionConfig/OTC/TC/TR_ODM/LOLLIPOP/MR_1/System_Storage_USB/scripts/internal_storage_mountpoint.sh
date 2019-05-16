#! /bin/bash
# Test case: Storage.USB_InternalStorage_Check.mountpoint
# Area: System_Storage/USB
# Test type: Functional
# ET Project: OTC Android Testing
# Author: Diana Dumitrescu

# Prerequisites:
# - needs root for toolbox du command
# Warning! If the prerequisites aren't met the verification will fail the test.
if [ -z "$1" -o -z "$2" ]
then
        echo "Usage: ./internal_storage_mountpoint.sh <mounted_folder> <fstype>"
        exit 1
fi
# Prerequisites verification
x=$(adb root)
if [ "$x" != "adbd is already running as root" ]
then
        if [ -n "$x" ]
        then
                echo "FAIL"
                exit 1
        fi
fi

# Uses mountstats as auxiliary script and toolbox to use du command in android shell
a=$(adb shell df $1 | grep $1 | awk '{print $2}' | tr -d "G")

b=$(./scripts/mountstats.sh $1 $2)

# Compares the 2 floats since Bash doesn't work with floats
c=$(echo $a'>'"3" | bc -l)

# Result comparison
if [ $c = 1 ]
then
        if [ "$b" = "PASS" ]
        then
                echo "PASS"
                exit 0
        else
                echo "FAIL"
                exit 1
        fi
else
        echo "FAIL- must have at least 1.5GB, encouraged to have at least 3GB."
        exit 1
fi
