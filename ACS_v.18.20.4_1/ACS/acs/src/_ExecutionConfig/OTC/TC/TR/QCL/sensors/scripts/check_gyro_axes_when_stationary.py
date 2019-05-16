'''
    Copyright 2015 Android Open Source Project

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
'''

import sys
from sensors_util import get_sensor_hal_info
from sensors_util import poll_for_events
from util import print_test_result
from util import exit_test

arg = sys.argv
try:
    sensor_info = get_sensor_hal_info(arg[1])
    sensor_id = sensor_info["index"]
    sensor_type = sensor_info["type"]
except:
    print_test_result(False, "Sensor was not found")
    exit_test()

poll_seconds = 3
timeout = poll_seconds * 2 + 5
events_list = poll_for_events(str(sensor_id), 0, timeout, poll_seconds)
if (len(events_list) > 0):
    passed = True
    message = "There were recorded only 0 values for all of the gyroscope axes"
    unverified_events = len(events_list)
    for event in events_list:
        if event['data']['x'] != 0 or event['data']['y'] != 0 or event['data']['z'] != 0:
                passed = False
                message = "It was expected that the sensor would record only 0 values for the three axes. "
                message += "This is the recorded data for one of the events: (x: {}, y: {}, z: {})"
                message = message.format(event["data"]["x"],event["data"]["y"],event["data"]["z"]) 
                break;
        unverified_events -= 1
    if unverified_events == 0 or not passed:
        print_test_result(passed, message)
    else:
        print_test_result(False, "There was an error while verifying events data")
else:
    print_test_result(False, "Poll exceeded the timeout")
