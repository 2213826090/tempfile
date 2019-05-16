#!/bin/bash

<<COMMENT
    Copyright 2014 Android Open Source Project

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
COMMENT

adb shell touch /data/test.file
CMD=`adb shell ls /data`
VALID=`echo $CMD | grep test.file | wc -l`
if [ $VALID -eq 1 ]
then
    echo "PASS"
else
    echo $VALID
fi
