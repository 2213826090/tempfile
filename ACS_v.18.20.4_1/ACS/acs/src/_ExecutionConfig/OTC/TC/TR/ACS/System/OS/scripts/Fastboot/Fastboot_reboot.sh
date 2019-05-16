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

DEVICE_NUMBER=`adb devices | wc | awk {'print $1'}`
ADB_RETURN=`adb reboot bootloader`
if [ $? -eq 0 ]
then
    echo "Entering fastboot"
else
    exit
fi

while [ $COUNTER -gt 0 ];
do
    COUNTER=$((COUNTER - 1))
    FASTBOOT_RETURN=`fastboot devices`
    # check fastboot devices contains fastboot in output
    CONTAINS=`echo $FASTBOOT_RETURN | grep fastboot | wc -w`
    if [ $CONTAINS -gt 0 ]
    then
        echo "Fastboot devices found the device"
        FASTBOOT_RETURN=`fastboot reboot`
        # check fastboot reboot contains "finished." in output
        CONTAINS=`echo $FASTBOOT_RETURN | grep 'finished.' | wc -w`
        if [ $CONTAINS -ge 0 ]
        then
            while [ $COUNTER_BOOT -gt 0 ]
            do
                COUNTER_BOOT=$((COUNTER_BOOT - 1))
                sleep 1
                ADB_DEVICES=`adb devices | wc | awk {'print $1'}`
                if [ $ADB_DEVICES -ge $DEVICE_NUMBER ]
                then
                    echo "Adb devices works"
                    echo `adb devices`
                    echo "PASS"
                    exit
                fi
            done
        fi
    fi
    sleep 1
done
