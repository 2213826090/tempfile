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

ADB_RETURN=`adb reboot recovery`
if [ $? -eq 0 ]
then
    echo "Entering recovery console"
else
    echo "Can't enter recovery console"
    exit
fi

while [ $COUNTER -gt 0 ]
do
    COUNTER=$((COUNTER - 1))
    ADB_DEVICES=`adb devices`
    # check adb devices contains recovery in output
    CONTAINS=`echo $ADB_DEVICES | grep recovery | wc -w`
    if [ $CONTAINS -gt 0 ]
    then
        echo "Adb devices found the device in recovery"
        echo "Sleeping for 5 seconds"
        sleep 5
        break
    else
        if [ $COUNTER -eq 1 ]
        then
            echo "Adb devices returned nothing. Maybe not in recovery."
            exit
        fi
    fi
    sleep 1
done

adb reboot
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

if [ $CONTAINS -gt 0 ]
then
    echo "PASS"
fi

