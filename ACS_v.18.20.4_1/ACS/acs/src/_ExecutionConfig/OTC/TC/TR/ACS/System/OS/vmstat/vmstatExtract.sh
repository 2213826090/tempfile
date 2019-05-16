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

vmstat="adb shell vmstat -n 1 > vmstat.txt"
timeout=30
parser=0

eval $vmstat
if [ $? -ne 0 ]
then
    echo "device not connected over adb"
    exit 0
fi

adb bugreport > /dev/null &
sleep 10

while [ $parser -le 50 ]
do
    sleep 1
    timeout=$((timeout-1))
    eval $vmstat
    parser=$(sh vmstatParser.sh vmstat.txt)
    if [ $timeout -lt 0 ]
    then
        echo "FAILURE"
        exit
    fi
done

adb reboot
echo "SUCCESS"
