#! /bin/bash

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

if [ ! -d "img" ];
then
    mkdir img

    size=10
    for num in `seq 5000`;
    do
       mod=$((num % 500))
       if [ $mod -eq 0 ]
       then
         size=$((size+10))
       fi
       convert -size ${size}x${size} -background white -fill black -gravity center caption:"Picture $num: $size x $size" img/img_${num}.png
       #echo $num
    done
fi

#adb push img /sdcard/Download/
#resync the download folder
adb shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/Download
if [ ! $? -eq 0 ]
then
    echo "FAILURE"
    exit
fi
sleep 30

#test gallery starts
adb shell am start com.google.android.gallery3d
if [ ! $? -eq 0 ]
then
    echo "FAILURE"
    exit
fi

sleep 10
activity_is_alive=`adb shell dumpsys window windows | grep gallery | wc | awk {'print $1'}`
if [ $activity_is_alive -eq 0 ]
then
    echo "FAILURE"
fi

adb shell am force-stop com.google.android.gallery3d
echo "SUCCESS"
