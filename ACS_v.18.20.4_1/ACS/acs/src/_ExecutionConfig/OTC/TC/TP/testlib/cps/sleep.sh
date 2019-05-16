#!/system/bin/sh

sleep 5
if dumpsys power | grep mScreenOn=true 
then
    echo press power
    input keyevent KEYCODE_POWER
fi


