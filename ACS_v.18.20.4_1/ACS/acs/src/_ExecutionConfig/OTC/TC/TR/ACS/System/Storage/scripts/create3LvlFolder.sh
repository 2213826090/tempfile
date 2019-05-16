#! /bin/bash -x
# Test case: Storage.USB_SDCard_CreateFolderWith255charsName
# Area: System_Storage/USB
# Test type: Reliability - Iterative
# ET Project: OTC Android Testing
# Author: Diana Dumitrescu (dmdumitx)

# auxiliary function

output0=$(adb shell rm -rf /storage/emulated/test*)

output1a=$(adb shell mkdir /storage/emulated/test3)
output1b=$(adb shell mkdir /storage/emulated/test3/test2)
output1c=$(adb shell mkdir /storage/emulated/test3/test2/test1)

if [[ $output1c ]]
then
        echo "FAIL - Mkdir command failed"
fi

output2=$(adb shell dd if=/dev/urandom of=/storage/emulated/test3/test2/test1/testfile count=128 bs=128)
output2a=$(adb shell ls /storage/emulated/test3/test2/test1/testfile)
if [[ -z $output2a ]]
then
        echo "FAIL - File create command failed"
fi

output3=$(adb shell cp -r /storage/emulated/test3 /storage/emulated/test3copy)
if [[ $output3 ]]
then
        echo "FAIL - File cp command failed"
fi

output4=$(adb shell cp -r /storage/emulated/test3copy /storage/emulated/test3copycopy)
if [[ $output4 ]]
then
        echo "FAIL - File cp de cp command failed"
fi

output5=$(adb shell cp -r /storage/emulated/test3copycopy /storage/emulated/test3copy)
if [[ $output5 ]]
then
        echo "FAIL - File cp back to cp command failed"
fi

# compare a to b
output6a=$(adb shell cmp -l /storage/emulated/test3/test2/test1/testfile /storage/emulated/test3copy/test2/test1/testfile)
output6b=$(adb shell cmp -l /storage/emulated/test3/test2/test1/testfile /storage/emulated/test3copycopy/test2/test1/testfile)
output6c=$(adb shell cmp -l /storage/emulated/test3copy/test2/test1/testfile /storage/emulated/test3copycopy/test2/test1/testfile)
if [[ $output6a ]]
then
        echo "FAIL - Files are not the same"
        exit 1
else
        if [[ -z $output6b ]]
        then
                if [[ $output6c ]]
                then
                        echo "FAIL"
                        exit 1
                else
                        echo "PASS"
                fi
        else
                echo "FAIL"
                exit 1
        fi
fi
