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

cmd='adb shell ps | grep chrome | awk "NR==1{print;exit}" | awk {'\''print $2'\''}'

adb logcat -c

word=`awk -v y=$((i*15)) "NR==y{print;exit}" /usr/share/dict/words`;
echo $word;
adb shell am start -a android.intent.action.VIEW -d "http://www.youtube.com/watch?v=FvHiLLkPhQE" ;
sleep 20;
#Click on the video to start playing
adb shell input tap 300 300
cmd_out=`eval $cmd`
pid=$cmd_out
echo "PID $pid"
hour='date +%H'
day='date +%w'
current_hour=`eval $hour`
current_day=`eval $day`
echo "day: $current_day hour: $current_hour "
#run for 24h
while [ $((`eval $hour` -1)) != $current_hour ] && [ $((`eval $day` -1)) != $current_day ];
do
    cmd_out=`eval $cmd`
    if [[ -z $cmd_out ]]
    then
        exit
    else
        if [ $pid == $cmd_out ]
            then
                echo "PID identical"
                #Verify every min if the chrome pid changed or is not present
                sleep 60
            else
                echo "PID diff $pid eval $cmd_out. Tested failed"
                exit
        fi
    fi
done
echo "Tested pass"
