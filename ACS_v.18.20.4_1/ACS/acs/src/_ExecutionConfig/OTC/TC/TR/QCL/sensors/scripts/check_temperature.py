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
import numpy
from sensors_util import poll_for_events
from sensors_util import get_sensor_hal_info
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

min_temp = arg[2]
max_temp = arg[3]
poll_seconds = 5
timeout = poll_seconds * 2 + 5
events_list = poll_for_events(sensor_id, 0, timeout, poll_seconds)
if (len(events_list) > 0):
    values = []
    for event in events_list:
        if event['type'] == sensor_type:
            values.append(event["data"]["temperature"])
    if len(values) > 0:
        mean_value = numpy.mean(values)
        relative_standard_deviation = numpy.std(values) / mean_value
        mean_value /= 1000
        message = "Expected temperature between " + min_temp + " and " + \
                  max_temp + ", obtained average temperature: " + str(mean_value) + \
                  ", relative standard deviation: " + str(relative_standard_deviation)
        if mean_value >= float(min_temp) and mean_value <= float(max_temp) and \
                        relative_standard_deviation < 0.05:
            print_test_result(True, message)
        else:
            print_test_result(False, message)
    else:
        print_test_result(False, "No temperature event found")
else:
    print_test_result(False, "Poll exceeded the timeout")
