#! /bin/bash
# Test case: Storage.USB_SDCard_Check.Mountpoint
# Area: System_Storage/USB
# Test type: Functional Positive
# ET Project: OTC Android Testing
# Author: Diana Dumitrescu

# Prerequisites:
# - insert an SDcard into DUT
# - run adbd as root
# Warning! If the prerequisites aren't met the verification will fail the test.

# Auxiliary functions used for closing the Settings menu
function exit_fail {
        echo "FAIL"
        adb shell am force-stop com.android.settings
        exit 1
}

function exit_pass {
        echo "PASS"
        adb shell am force-stop com.android.settings
        exit 0
}

if [ -z "$1" -o -z "$2" -o -z "$3" -o -z "$4" ]
then
        echo "Usage: ./secondary_external_storage_mountpoint.sh <mounted_folder_1> <fstype1> <mounted_folder2> <fstype2>"
        exit 1
fi

# Uses mountstats.sh as an auxiliary script. The verification takes place into
# this script.
a=$(bash ./scripts/mountstats.sh $1 $2)
b=$(bash ./scripts/mountstats.sh $3 $4)
if [ $a != "PASS" ]
then
        if [ $b != "PASS" ]
        then
                echo "Mount folders and their fstypes not found or are different from the test case!"
                echo "FAIL"
                exit 1
        else
                echo $1" mount folder does not have the "$2" fstype!"
                echo "FAIL"
                exit 1
        fi
fi

# UI verification
param1total=$(adb shell df | grep $1 | awk '{print $2}' | tr -d "G")
param2total=$(adb shell df | grep $3 | awk '{print $2}' | tr -d "G")
param1free=$(adb shell df | grep $1 | awk '{print $4}' | tr -d "G")
param2free=$(adb shell df | grep $3 | awk '{print $4}' | tr -d "G")

# Start the Settings->Storage menu
output1=$(adb shell am start -a android.settings.MEMORY_CARD_SETTINGS | awk '{print $1}' | grep "Starting:")
if [ -z $output1 ]
then
        echo "The Storage menu cannot be loaded!"
        echo "FAIL"
        exit 1
fi

# Verify the output from storage.py
output2=$(python ./scripts/storage.py | tr -d "\r")
output3=$(echo $output2 | tr -d "\n")
if [[ $output3  = "Settings app not opened" ]]
then
        echo "The DUT is not awake or the graphical interface is not the one needed!"
        exit_fail
fi
if [[ $output3 = "Total space value not found in Settings" ]]
then
        echo "Graphical interface error! Total space not found."
        exit_fail
fi
if [[ $output3 = "Timeout reached, still calculating values in Settings" ]]
then
        echo "Graphical interface error! Still calculating values."
        exit_fail
fi
if [[ $output3 = "Available space value not found in Settings" ]]
then
        echo "Graphical interface error! Available space not found."
        exit_fail
fi

# Parse the output to obtain the values
interfacetotal=$(echo $output2 | awk '{print $1}' | tr -d "GB")
interfacefree=$(echo $output2 | awk '{print $2}' | tr -d "GB")

# Compare the outputs from the mountpoints offered as parameters
comp1=$(echo $param1total"=="$param2total | bc -l)
comp2=$(echo $param1free"=="$param2free | bc -l)

# Result comparison: the difference between the interface value and the
# df value must be 0.05 maximum.
if [ $comp1 = 1 -a $comp2 = 1 ]
then
        aux=$(echo $param1total'-'$interfacetotal | bc -l)
        aux1=$(echo $param1free'-'$interfacefree | bc -l)
        comp3=$(echo ${aux#-}'<='"0.05" | bc -l)
        comp4=$(echo ${aux1#-}'<='"0.05" | bc -l)
        if [ $comp3 = 1 -a $comp4 = 1 ]
        then
                exit_pass
        else
                echo "Dimensions differ! The interface does not show the right value"
                exit_fail
        fi
else
        echo "Dimensions differ! Mountpoints return different values!"
        exit_fail
fi
