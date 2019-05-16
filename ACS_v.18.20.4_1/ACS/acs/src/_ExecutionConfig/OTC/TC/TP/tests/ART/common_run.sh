#!/bin/bash

PACKAGE=$1
TEST_CASE=$2
#TEST_CASE=`echo $0|sed 's/.sh//g'`
echo $TEST_CASE
if [ -z "$ANDROID_SERIAL" ]
then
    ANDROID_SERIAL=$(adb devices | egrep 'device$' | head -n1 | awk '{print $1}')
fi
echo $ANDROID_SERIAL
echo

cd $(dirname $0)
cd $PACKAGE
#export ANDROID_SDK_ROOT="$(pwd)/sdk"
#echo "Current directory: $(pwd)"
RUN_CMD="./run.sh"
chmod a+x $RUN_CMD

# Run all tests
BIT_TYPE=`adb -s $ANDROID_SERIAL shell getprop ro.product.cpu.abi | grep "x86_64"`
if [ -z "$BIT_TYPE" ]
then
    BIT="32"
else
    BIT="64"
fi

RUN_TIME=`adb -s $ANDROID_SERIAL shell getprop | grep persist.sys.dalvik.vm.lib | awk '{ print $2}'| tr -d '\r'`
if [ "$RUN_TIME" == "[libdvm.so]" ]
then
    ART_DALVIK="dalvik"
else
    ART_DALVIK="art"
fi

if [ "$ART_DALVIK" == "dalvik" ]
then
    if [ "$BIT" == "32" ]
    then
        echo "Testing dalvik32..."
        $RUN_CMD ANDROID_SERIAL=$ANDROID_SERIAL $TEST_CASE
    else
        echo "Testing dalvik64..."
        $RUN_CMD VM_RUNTIME=DVM VM_COMMAND=dalvikvm64 ANDROID_SERIAL=$ANDROID_SERIAL $TEST_CASE
    fi
else
    if [ "$BIT" == "32" ]
    then
        echo "Testing ART32..."
        $RUN_CMD VM_RUNTIME=ART ANDROID_SERIAL=$ANDROID_SERIAL $TEST_CASE
    else
        echo "Testing ART64..."
        $RUN_CMD VM_RUNTIME=ART64 VM_COMMAND=dalvikvm64 ANDROID_SERIAL=$ANDROID_SERIAL $TEST_CASE
    fi
fi

# cd ../
#rm -rf ART_tests
