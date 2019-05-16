#! /bin/bash
# Test case: Storage.USB_EmulatedStorage_Check.mountpoint
# Area: System_Storage/USB
# Test type: Functional
# ET Project: OTC Android Testing
# Author: Diana Dumitrescu

# Prerequisites:
# - run adbd as root
# - no other specifications required
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

# Verifying the arguments
if [ $# -ne 8 ]
then
        echo "Usage: ./emulated_storage_mountpoints.sh <folder_path_1> <simlink_1> <simlink_folder_1> <folder_path_2> <simlink_2> <simlink_folder_2> <mounted_folder> <fstype>"
        exit 1
fi

# Uses mounted_simlinks.sh and mountstats.sh as auxiliary scripts
a=$(./scripts/simlinks.sh $1 $2 $3)
b=$(./scripts/simlinks.sh $4 $5 $6)
c=$(./scripts/mountstats.sh $7 $8)

if [[ $a != "PASS" ]]
then
        if [[ $b != "PASS" ]]
        then
                if [[ $c != "PASS" ]]
                then
                        echo "The simlinks are not pointing to the right folders and the mountpoint is not mounted with the correct fstype!"
                        echo "FAIL"
                        exit 1
                else
                        echo "The simlinks are not pointing to the right folders!"
                        echo "FAIL"
                        exit 1
                fi
        else
                echo "$1 simlink is not pointing to the right folder or the folder_path_1 is not the right one!"
                echo "FAIL"
                exit 1
        fi
fi

# Getting the total space and free space from internal shell commands
insidetotal=$(adb shell df | grep $7 | awk '{print $2}' | tr -d "G")
insidefree=$(adb shell df | grep $7 | awk '{print $4}' | tr -d "G")

# Start the Settings->Storage menu
output1=$(adb shell am start -a android.settings.MEMORY_CARD_SETTINGS | awk '{print $1}' | grep "Starting:")
if [ -z $output1 ]
then
        echo "The Storage menu cannot be loaded!"
        echo "FAIL"
        exit 1
fi

# Interrogate the interface for total space and available space
output2=$(python ./scripts/emulated_storage.py | tr -d "\r")
output3=$(echo $output2 | tr -d "\n")

# Verify if there is any error message returned from the script
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

# Result comparison
aux=$(echo $insidetotal'-'$interfacetotal | bc -l)
aux1=$(echo $insidefree'-'$interfacefree | bc -l)
comp1=$(echo ${aux#-}'<='"0.05" | bc -l)
comp2=$(echo ${aux1#-}'<='"0.05" | bc -l)
if [ $comp1 = 1 -a $comp2 = 1 ]
then
        exit_pass
else
        echo "The interface does not show the right values!"
        exit_fail
fi
