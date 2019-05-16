#!/bin/bash

# This utility is used to deployment content files into
#    Android device.

current_dir=$PWD
content_zip="$PWD/mtbf-content.zip"
ADB_SERIAL=''
while getopts s:f:h OPTION
do
    case $OPTION in
    s) ADB_SERIAL=$OPTARG;;
    f) content_zip=$OPTARG;;
    ?) usage; exit;;
    esac
done

function usage()
{
echo "$0 [-s serial][-f content_zip][-n]"
}

function repeat_cmd()
{
    for ((i=0; i<5; i++))
    do
        eval $1 && return
    done
    return 1
}

ADB_OPTIONS=''
if [ ! -z "$ADB_SERIAL" ]
then
    ADB_OPTIONS="$ADB_OPTIONS -s ${ADB_SERIAL}"
fi

if [ ! -f "$content_zip" ]
then
    echo "$content_zip does not exist!"
    exit
fi
cd $(dirname $content_zip)
unzip -n $content_zip

if [ ! -d mtbf_content ]
then
    echo "No mtbf_content file found."
    exit
fi
cd mtbf_content

if [ -d "video/" ]
then
    echo -n "Deployment video content files into Android device ... "
    CMDSTR="adb $ADB_OPTIONS push video/ /mnt/sdcard/Movies/ &> /dev/null"
    repeat_cmd "$CMDSTR" && echo "done." || echo "failed."
fi

if [ -d "Music/" ]
then
    echo -n "Deployment Music content files into Android device ... "
    CMDSTR="adb $ADB_OPTIONS push Music/internal_storage_music/ /mnt/sdcard/Music/ &> /dev/null"
    repeat_cmd "$CMDSTR" && echo "done." || echo "failed."
fi

if [ -d "Pictures/" ]
then
    echo -n "Deployment Pictures content files into Android device ... "
    CMDSTR="adb $ADB_OPTIONS push Pictures/ /mnt/sdcard/Pictures/ &> /dev/null"
    repeat_cmd "$CMDSTR" && echo "done." || echo "failed."
fi

if [ -d "files/" ]
then
    echo -n "Deployment content files into Android device ... "
    CMDSTR="adb $ADB_OPTIONS push files/ /sdcard/Download &> /dev/null"
    repeat_cmd "$CMDSTR" && echo "done." || echo "failed."
fi

if [ -d "Apk/" ]
then
    echo -n "Deployment Apk files into Android device ... "
    CMDSTR="adb $ADB_OPTIONS push Apk/ /sdcard/Apk &> /dev/null"
    repeat_cmd "$CMDSTR" && echo "done." || echo "failed."
fi

echo -n "trigger music files to show on Android device ... "
CMDSTR="adb $ADB_OPTIONS shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///mnt/sdcard/Music &> /dev/null"
repeat_cmd "$CMDSTR" && echo "done." || echo "failed."

echo -n "trigger video files to show on Android device ... "
CMDSTR="adb $ADB_OPTIONS shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:////mnt/sdcard/Movies/ &> /dev/null"
repeat_cmd "$CMDSTR" && echo "done." || echo "failed."

echo -n "install new newPSI_Recorder.apk ... "
CMDSTR="adb $ADB_OPTIONS install -r Apk/newPSI_Recorder.apk &> /dev/null"
repeat_cmd "$CMDSTR" && echo "done." || echo "failed."

