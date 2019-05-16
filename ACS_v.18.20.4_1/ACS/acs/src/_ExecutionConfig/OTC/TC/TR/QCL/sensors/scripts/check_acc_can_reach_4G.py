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
from sensors_util import get_sensor_hal_info
from sensors_util import poll_for_events
from util import print_test_result
from util import validate_result
from util import menu
from util import exit_test

def check_acc_coords_reach_4G(args):
    global VERDICT, OUTPUT
    values = []
    poll_time = 3
    # an approximation for 4G
    acc = 39
    timeout = poll_time * 2 + 5
    print "Press ENTER to start the record"
    while (True):
        input_line = raw_input("")
        if input_line == "":
            events = poll_for_events(sensor_id, 0, timeout, poll_time)
            if len(events) > 0:
                break
            else:
                print_test_result(False, "No data results from sensor poll")
                VERDICT = FAILURE
                return
    for event in events:
        values.append(event['data'][AXIS])
    max_value = max(values)
    message = "max value expected over {} axis: {}, obtained max value: {}"
    message = message.format(AXIS, acc, max_value)
    if max_value > acc:
        print_test_result(True, message)
        VERDICT = SUCCESS
    else:
        print_test_result(False, message)
        VERDICT = FAILURE
    OUTPUT = message
# test execution
VERDICT = FAILURE
OUTPUT = ""
AXIS = TC_PARAMETERS('PARAMETERS')
COMP_CONF = TC_PARAMETERS('COMP_CONF')
COMP_CONF = COMP_CONF[1:-1]

try:
    sensor_info = get_sensor_hal_info(COMP_CONF)
    sensor_id = sensor_info["index"]
    sensor_type = sensor_info["type"]
except:
    OUTPUT = "Sensor was not found"
    print_test_result(False, OUTPUT)
    exit_test()
message = "Move the device fast over the {} axis".format(AXIS)
menu_items = ["Press enter when you are ready to begin the test",
              {message: check_acc_coords_reach_4G},
              {"Do you want repeat the test (y/n)": validate_result}]

menu(menu_items, menu_items[1:])
