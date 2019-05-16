<<COMMENT
    Copyright 2014 Android Open Source Project

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
COMMENT

COUNTER=500
COUNTER_BOOT=500
COUNTER_REBOOT=1

DEVICE_NUMBER=`adb devices | wc | awk {'print $1'}`
ADB_RETURN=`adb reboot bootloader`
if [ $? -eq 0 ]
then
    echo "Entering fastboot"
else
    echo "Can't enter fastboot"
    exit
fi

while [ $COUNTER -gt 0 ]
do
    COUNTER=$((COUNTER - 1))
    FASTBOOT_RETURN=`fastboot devices`
    # check fastboot devices contains fastboot in output
    CONTAINS=`echo $FASTBOOT_RETURN | grep fastboot | wc -w`
    if [ $CONTAINS -gt 0 ]
    then
        echo "Fastboot devices found the device"
        break
    else
        if [ $COUNTER -eq 1 ]
        then
            echo "Fastboot devices returned nothing. Maybe not in fastboot."
            exit
        fi
    fi
    sleep 1
done

#already unlocked. Lock and unlock
echo "Fastboot oem lock"
FASTBOOT_RETURN=`fastboot oem lock 2>&1`
sleep 1
CONTAINS=`echo $FASTBOOT_RETURN | grep 'finished.' | wc -l`
if [ $CONTAINS -eq 1 ]
then
    FASTBOOT_RETURN=`fastboot oem unlock 2>&1`
    sleep 1
    CONTAINS=`echo $FASTBOOT_RETURN | grep '(bootloader) Nothing to do.' | wc -l`
    if [ $CONTAINS -eq 0 ]
    then
        #Check format erase misc is possible
        VALIDATION=`fastboot erase misc 2>&1`
        sleep 1
        CONTAINS=`echo $VALIDATION | grep 'bootloader must not be locked' | wc -l`
        if [ $CONTAINS -eq 1 ]
        then
            echo "After fastboot oem unlock can't execute fastboot erase misc"
            fastboot reboot
            exit
        else
            fastboot oem lock
            sleep 1
            fastboot reboot
            echo "Rebooting"
                while [ true ]
                do
                    CMD=`adb get-state`
                    VALIDATION=`echo $CMD | grep device | wc -l`
                    echo -n "."
                    sleep 1
                    if [ $VALIDATION -eq 1 ]
                    then
                        while [ true ]
                        do
                            CMD=`adb shell getprop sys.boot_completed`
                            VALIDATION=`echo $CMD | grep 1 | wc -l`
                            echo -n "."
                            sleep 1
                            if [ $VALIDATION -eq 1 ]
                            then
                                break
                            fi
                        done
                        break
                    fi
                done
            echo "PASS"
        fi
    fi
fi

