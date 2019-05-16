#!/bin/bash

BUILD_KEYS=`adb shell cat adb_keys | grep buildbot`

TEST_KEYS=`adb shell cat adb_keys | grep "testbot@testbot\.intel\.com"`

GOOGLE_KEYS=`adb shell cat adb_keys | grep "@google.com"`

if [ "$BUILD_KEYS" ]; then
    echo "Build key:"
    echo $BUILD_KEYS

    if [ "$TEST_KEYS" ]
    then
        echo "Test key:"
        echo $TEST_KEYS

        if [ "$GOOGLE_KEYS" ]
        then
            echo "Google key:"
            echo $GOOGLE_KEYS
            echo "PASS"
        else
            echo "No Google key"
            exit
        fi

    else
        echo "No test key"
        exit
    fi

else
    echo "No build key"
    exit
fi
