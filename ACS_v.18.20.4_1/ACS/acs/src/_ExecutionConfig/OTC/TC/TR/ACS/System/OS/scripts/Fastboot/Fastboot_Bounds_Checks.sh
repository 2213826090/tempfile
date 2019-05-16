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

reboot()
{
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
}


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

dd if=/dev/zero of=/tmp/file.tmp bs=1k count=10000

CMD=`fastboot oem unlock 2>&1`
CONTAINS=`echo $CMD | grep 'finished.' | wc -l`
if [ $CONTAINS -eq 1 ]
then
    CMD=`fastboot flash misc /tmp/file.tmp 2>&1`
    echo $CMD
    CONTAINS=`echo $CMD | grep 'FAILED (remote: target partition too small!)' | wc -l`
    if [ $CONTAINS -eq 1 ]
    then
        fastboot oem lock
        reboot
        echo "PASS"
    else
        reboot
    fi
fi
