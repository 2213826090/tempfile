#! /bin/bash
#Storage.USB_SDCard_Format
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

# Check that sdcard is inserted and mounted
check_mounted

# Start the Settings->Storage menu
step1=$(adb shell am start -a android.settings.MEMORY_CARD_SETTINGS | awk '{print $1}' | grep "Starting:")
if [ -z $step1 ]
then
        echo "FAIL - The Storage menu cannot be loaded!"
        adb shell am force-stop com.android.settings
        exit 1
fi

# Erase SDcard from UI
error=$(python ./scripts/UIFormatSDcard.py 2>&1)
if [[  "$error" != "" ]]
then
        echo "FAIL - Erasing from UI failed with " $error
        adb shell am force-stop com.android.settings
        exit 1
fi

# Check that the SDcard was remounted, wait another 5 seconds in case SDcard is large
sleep 5
check_mounted

# Check that the SDcard was exported as SECONDARY_EXTERNAL_STORAGE
step4=$(adb shell ls /storage/sdcard1/ | grep Android)
if [ -z $step4 ]
then
        echo "FAIL - SDcard was NOT exported as SECONDARY_EXTERNAL_STORAGE - external storage access tests will fail"
        adb shell am force-stop com.android.settings
        exit 1
fi

# Check that the SDcard was mounted read only
# For now this will be checked on external storage access tests

#Close the Settings application
adb shell am force-stop com.android.settings
echo "PASS"
