#! /bash/sh

x=$1
y=$2

#adb  shell am force-stop com.android.settings
#adb  shell am start -a android.settings.APPLICATION_DEVELOPMENT_SETTINGS
sleep 2
input tap x y

devices
sleep 5
input tap x y

input keyevent 22
input keyevent 22
 input keyevent 22
sleep 2
input keyevent 66
 input keyevent 66
