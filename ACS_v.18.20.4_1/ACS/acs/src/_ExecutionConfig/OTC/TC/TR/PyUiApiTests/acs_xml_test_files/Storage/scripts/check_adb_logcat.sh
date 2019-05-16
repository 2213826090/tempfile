#!/bin/bash

ADBLOG=`adb logcat -d`

if [ -z "$ADBLOG" ] ; then
    echo "Fail"
else
    echo "Pass"
fi