#! /bin/bash
# This script searches the content of the /proc/self/mountstats file and returns if there
# is a mount point with the specified mount folder (param 1) and with the provided fstype
# (param 2).
# This is an auxiliary script.
# Author: Diana Dumitrescu

# This part verifies the correct usage of the script
if [ -z "$1" -o -z "$2" ]
then
        echo "Usage: ./mountstats.sh <mounted_folder> <fstype>"
        exit 1
fi

# The content of the file is searched for the mount folder (param 1) and from the output
# the fstype is extracted
a=$(adb shell cat /proc/self/mountstats | grep $1' ' | awk '{print $5}' | tr -d '\r')
b=$(adb shell cat /proc/self/mountstats | grep $1' ' | awk '{print $8}' | tr -d '\r')

# The results are compared
if [ -z "$a" -o -z "$b" ]
then
        echo "FAIL"
        exit 1
else
        if [ $a = $1 -a $b = $2 ]
        then
                echo "PASS"
                exit 0
        else
                echo "FAIL"
                exit 1
        fi
fi

