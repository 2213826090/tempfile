#/bin/bash

RES=`adb shell ls`

if [ -z "$RES" ] ; then
    echo "Fail"
    exit 0
fi

INITIAL=`adb shell pwd`

FINAL=`adb shell "cd etc; pwd"`

FINALSIZE=`echo "${#FINAL}-1" | bc`
INITIALSIZE=`echo "${#INITIAL}-1" | bc`


INITIAL=`echo $INITIAL | cut -c 1-$INITIALSIZE`
FINAL=`echo $FINAL | cut -c 1-$FINALSIZE`

if [ "${INITIAL}" == "/" ] ; then
    INITIAL=""
fi


if [ "${INITIAL}/etc" == "$FINAL" ] ; then
    echo "equal" >/dev/null
else
   echo "Fail"
   exit 0
fi

DMESGOUTPUT=`adb shell dmesg`

if [ -z "$DMESGOUTPUT" ] ; then
    echo "Fail"
    exit 0
fi

ADBLOG=`adb logcat -d`

if [ -z "$ADBLOG" ] ; then
    echo "Fail"
else
    echo "Pass"
fi
