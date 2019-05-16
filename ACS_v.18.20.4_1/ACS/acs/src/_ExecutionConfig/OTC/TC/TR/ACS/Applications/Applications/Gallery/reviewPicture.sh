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

convert -size 100x100 -background white -fill black -gravity center caption:"Picture Test" img.png

adb push img.png /sdcard/
if [ ! $? -eq 0 ]
then
    echo "FAILURE"
    exit
fi

#resync the download folder
adb shell am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file:///sdcard/img.png
if [ ! $? -eq 0 ]
then
    echo "FAILURE"
    exit
fi

sleep 10

#test gallery starts
adb shell am start -a android.intent.action.VIEW -d file:///sdcard/img.png -t image/png -n com.google.android.gallery3d/com.android.gallery3d.app.GalleryActivity
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
