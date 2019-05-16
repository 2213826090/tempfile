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
import os
from sensors_util import define_thresholds
from sensors_util import get_period_values
from sensors_util import poll_for_events
from sensors_util import get_sensor_hal_info
from util import is_frequency_in_expected_range
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

greater = False
if (arg[2] == "min"):
    delay = sensor_info["maxDelay"]
    sensor_frequency = 1000000.00 / delay
elif (arg[2] == "max"):
    delay = sensor_info["minDelay"]
    sensor_frequency = 1000000.00 / delay
else:
    sensor_frequency = int(arg[2])
    greater = True

if sensor_frequency < 1:
    sensor_frequency = 1

poll_seconds = define_thresholds(sensor_frequency)
timeout = poll_seconds * 2 + 5

if greater:
    events_list = poll_for_events(sensor_id, 0, timeout, poll_seconds)
else:
    events_list = poll_for_events(sensor_id, sensor_frequency, timeout, poll_seconds)

if (len(events_list) > 0):
    periods = get_period_values(events_list, sensor_type)
    result = is_frequency_in_expected_range(sensor_frequency, periods, greater)
    print_test_result(result["test_passed"], result["message"])
else:
    print_test_result(False, "Poll exceeded the timeout")
