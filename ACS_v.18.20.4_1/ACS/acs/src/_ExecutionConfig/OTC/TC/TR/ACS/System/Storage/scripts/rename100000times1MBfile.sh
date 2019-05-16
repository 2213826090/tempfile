#!/bin/bash
# prototype: ./rename10000times1MBfile.sh <path_from_device> <no_of_iterations>

# @param: $1 - path
function create_file()
{
    adb shell dd if=/dev/zero of=$1/original count=1024 bs=1024 &> /dev/null
    local output1=$(adb shell ls -l $1 | grep original)
    if [ -n "$output1" ]
    then
        echo "An error occured: Original file not found or not created. FAIL"
        exit 1
    fi
}

# @param: $1 - path
# @return: 0 - PASS, 1 - CopyA not found, 2 - CopyB not found, 3 - Comparison failed
function rename_file()
{
    adb shell cp $1/original $1/copyA
    local output2=$(adb shell ls -l $1 | grep copyA)
    if [ -n "$output2" ]
    then
        echo 1
    fi
    adb shell mv $1/copyA $1/copyB
    local output1=$(adb shell ls -l $1 | grep copyB)
    if [ -n "$output1" ]
    then
        echo 2
    fi
    local md5original=$(adb shell md5sum $1/original | awk '{print$1}')
    local md5copyB=$(adb shell md5sum $1/copyB | awk '{print$1}')
    if [[ "$md5original" != "$md5copyB" ]]
    then
        echo 3
    else
        echo 0
    fi
    adb shell rm $1/copyB
}

# @param: $1 - error_code
function interpret_debug_msg()
{
    if [ $1 -eq 1 ]
    then
        echo "An error occured: CopyA file not found. Unable to copy file. FAIL"
    fi
    if [ $1 -eq 2 ]
    then
        echo "An error occured: CopyB file not found. Unable to rename file. FAIL"
    fi
    if [ $1 -eq 3 ]
    then
        echo "CopyB file is not the same as original file. FAIL"
    fi
}

# cleanup function
function cleanup()
{
    adb shell rm $1/original &> /dev/null
    adb shell rm $1/copyA &> /dev/null
}

create_file $1
passCount=0
LIMIT=$2
for ((a=1; a <= LIMIT; a++))
do
    echo "Rename test $a:"
    step_res=$(rename_file $1)
    if [ $step_res -eq 0 ]
    then
        passCount=$(echo "$passCount+1" | bc -l)
        echo "PASS"
    else
        interpret_debug_msg $step_res
    fi
done
echo "PASS rate: $passCount out of $2 tests"
if [ $passCount -ne $2 ]
then
    echo "Test result: FAIL"
else
    echo "Test result: PASS"
fi
cleanup $1
echo "$(($SECONDS / 3600)) hours, $(($SECONDS / 60)) minutes, $(($SECONDS % 60)) seconds elapsed."
