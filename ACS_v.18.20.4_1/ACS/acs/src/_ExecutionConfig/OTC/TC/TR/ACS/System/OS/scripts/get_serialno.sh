#!/bin/bash
tmp=$(adb get-serialno)
tmp2=$(adb devices | awk 'NR == 2 { print $1 }')

if [ "$tmp" == "$tmp2" ]; then
        echo "PASS"
fi