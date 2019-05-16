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

#print for debug
fastboot getvar all
DIR="$(cd "$(dirname "$0")" && pwd)"
TEMPLATE_DIR=$DIR"/"$1
echo "Template dir "$TEMPLATE_DIR
VALID=0;
LINES_TO_BE_VALID=`cat $DIR"/"$1 | wc -l`
GETVARS=`fastboot getvar all 2>&1`;
while read line;
do
    if [ `echo $GETVARS | grep "$line" | wc -l` -eq 1 ];
    then
        VALID=$((VALID+1));
    else
        echo "Var not found in fastboot getvar all command: "$line
    fi
done < $TEMPLATE_DIR
echo "VALID lines: "$VALID
echo "Total number of lines: "$LINES_TO_BE_VALID
if [ $VALID -eq $LINES_TO_BE_VALID ] && [ $VALID -ne 0 ]
then
    echo "PASS"
fi
reboot
