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

import os
import sys
from sensors_util import get_sensor_hal_info
from sensors_util import poll_for_events
from sensors_util import get_axis_values
from sensors_util import is_sd_as_expected
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

axis = arg[2]
sensor_freq = 1000000 / sensor_info["maxDelay"]
poll_time = 3
timeout = poll_time * 2 + 5
events_list = poll_for_events(sensor_id, sensor_freq, timeout, poll_time)

if (len(events_list) > 0):
    values = get_axis_values(axis, events_list, sensor_type)
    result = is_sd_as_expected(values)
    print_test_result(result["test_passed"], result["message"])
else:
    print_test_result(False, "Poll exceeded the timeout")
