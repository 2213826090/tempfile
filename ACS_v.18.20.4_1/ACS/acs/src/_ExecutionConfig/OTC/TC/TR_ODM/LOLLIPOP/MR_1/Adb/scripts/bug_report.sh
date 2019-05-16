#!/bin/bash
TMP_LOCATION=/tmp/bugreport.tmp
adb bugreport > ${TMP_LOCATION}
result=$(tail -2 ${TMP_LOCATION} | head -1 | awk '{print $3}')
echo ${result}
if [ "done"="$result" ]; then
    echo 'PASS'
else
    echo 'FAIL'
fi
rm $TMP_LOCATION