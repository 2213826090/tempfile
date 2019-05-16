#!/bin/bash

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

adb root
sleep 3
adb remount /system
sleep 1
adb shell "rm -rf /cache/pstore/"
sleep 3
adb shell "mv /system/bin/tm_pstore_probe /system/bin/tm_pstore_probe.back"
sleep 3
adb shell "ls /cache/pstore/"
adb shell "echo c > /proc/sysrq-trigger"
sleep 20
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
                adb root
                sleep 3
                adb remount
                sleep 1
                CMD=`adb shell grep -r "Kernel panic" /cache/`
                echo $CMD
                if [ `echo $CMD | wc -l` -ge 1 ]
                then
                    echo "PASS"
                fi
                adb shell "mv /system/bin/tm_pstore_probe.back /system/bin/tm_pstore_probe"
                sleep 1
                exit
            fi
        done
        break
    fi
done
