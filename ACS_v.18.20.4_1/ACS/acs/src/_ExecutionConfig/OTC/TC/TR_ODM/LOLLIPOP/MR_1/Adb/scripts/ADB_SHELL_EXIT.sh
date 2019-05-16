#!/bin/bash

OK_MESSAGE="You are now connected!"
BAD_MESSAGE="You should not see this message!!!"

TEST=`adb shell "echo -n '$OK_MESSAGE'; exit; echo ' $BAD_MESSAGE'"`
echo $TEST
VALIDATION=`echo $TEST | grep "$BAD_MESSAGE" | wc -l`
if [ $VALIDATION -eq 0 ]
then
        echo "PASS"
else
        echo "adb shell did not exit"
fi