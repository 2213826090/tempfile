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

for i in {1..30}
do
    adb logcat -c
    echo $i
    word=`awk -v y=$((i*15)) "NR==y{print;exit}" /usr/share/dict/words`;
    echo $word;
    #adb shell am start -a android.intent.action.VIEW -d "https://m.youtube.com/results?q=$word " ;
    adb shell am start -a android.intent.action.VIEW -d "https://www.google.ro/?s#q=$i $word " ;
    #adb shell ps | grep chrome
    echo "PID $pid"
    cmd_out=`eval $cmd`
    if [ $i -eq 1 ]; then
        pid=$cmd_out
    fi

    if [ $pid -eq $cmd_out ]
        then
            echo "PID identical"
        else
            echo "PID diff $pid eval $cmd_out. Tested failed"
            exit
    fi

    adb logcat -d | grep -n "Low on memory"
    sleep 7;
done
echo "Tested pass"
