#!/bin/bash

#:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
#The source code contained or described herein and all documents related
#to the source code ("Material") are owned by Intel Corporation or its
#suppliers or licensors. Title to the Material remains with Intel Corporation
#or its suppliers and licensors. The Material contains trade secrets and
#proprietary and confidential information of Intel or its suppliers and
#licensors.

#The Material is protected by worldwide copyright and trade secret laws and
#treaty provisions. No part of the Material may be used, copied, reproduced,
#modified, published, uploaded, posted, transmitted, distributed, or disclosed
#in any way without Intel's prior express written permission.

#No license under any patent, copyright, trade secret or other intellectual
#property right is granted to or conferred upon you by disclosure or delivery
#of the Materials, either expressly, by implication, inducement, estoppel or
#otherwise. Any license under such intellectual property rights must be express
#and approved by Intel in writing.

#:organization: INTEL OTC ANDROID QA
#:description: This test run "adb shell getevent" command and verifies that
#no events are registered for a specified amount of time.
#:since: 3/4/15
#:author: dguliex

if [ -z "$1" ] || [ "$1" -eq "0" ]; then
    echo "Usage: ./System_Touch_checkAdbShellGetEventOutputWhenScreenOnButNoTouchOperations.sh <time_to_wait_for_events>"
    exit 1
fi

GETEVENT_OUTPUT=$(timeout $1 adb shell getevent)
EVENT=$(echo "$GETEVENT_OUTPUT" | grep "/dev/input/event[0-9]:")

if [[ -n $EVENT ]]; then
    echo "FAIL"
    echo "$GETEVENT_OUTPUT"
    exit 1
else
    echo "PASS"
    exit 0
fi
