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

import numpy
import os
from sensors_util import get_sensor_hal_info
from sensors_util import poll_for_events
from util import validate_result
from util import menu
from util import print_test_result
from util import exit_test

poll_time = 3
timeout = poll_time * 2 + 5

def check_light_value_is_max(args):
    global VERDICT, OUTPUT
    values = []
    print "Press ENTER to start the record"
    while (True):
        input_line = raw_input("")
        if input_line == "":
            events = poll_for_events(sensor_id, 0, timeout, poll_time)
            break
    for event in events:
        values.append(event['data']['light'])
    mean_value = numpy.mean(values)
    relative_standard_deviation = numpy.std(values) / mean_value
    message = "desired_value: {}, obtained average: {}, relative standard deviation: {}"
    message = message.format(sensor_max_value, mean_value, relative_standard_deviation)
    if 0.95 * sensor_max_value < mean_value < 1.05 * sensor_max_value and \
                    relative_standard_deviation < 0.05:
        print_test_result(True, message)
        VERDICT = SUCCESS
    else:
        print_test_result(False, message)
        VERDICT = FAILURE
    OUTPUT = message
# test execution
VERDICT = FAILURE
OUTPUT = ""
COMP_CONF = TC_PARAMETERS('COMP_CONF')
COMP_CONF = COMP_CONF[1:-1]

try:
    sensor_info = get_sensor_hal_info(COMP_CONF)
    sensor_id = sensor_info["index"]
    sensor_type = sensor_info["type"]
    sensor_max_value = sensor_info["maxValue"]
except:
    OUTPUT = "Sensor was not found"
    print_test_result(False, OUTPUT)
    exit_test()

menu_items = ["Press enter when you are ready to begin the test",
              {"Put the device in a maximum light environment": check_light_value_is_max},
              {"Do you want repeat the test (y/n)": validate_result}]

menu(menu_items, menu_items[1:])
