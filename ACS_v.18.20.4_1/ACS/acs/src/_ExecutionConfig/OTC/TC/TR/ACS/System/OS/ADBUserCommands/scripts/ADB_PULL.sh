#!/bin/bash
adb shell touch /sdcard/test_file
test_dir=$(date "+%Y%m%d-%H.%M.%s")
mkdir $test_dir


output=$((adb pull /sdcard/test_file $test_dir/non_existing/) 2>&1)

expected="cannot create '$test_dir/non_existing/': Is a directory"

rm -rf $test_dir/
adb shell rm /sdcard/test_file

if [ "$output" == "$expected" ]; then
    echo "Success"
else
    echo "FAIL"
fi