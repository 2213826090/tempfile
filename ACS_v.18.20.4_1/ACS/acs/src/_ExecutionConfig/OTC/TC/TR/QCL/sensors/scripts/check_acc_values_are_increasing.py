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
from util import are_values_ordered
from util import validate_result
from util import menu
from util import eliminate_succesive_duplicates
from util import print_test_result
from util import exit_test

G = 9.8
poll_time = 5
timeout = poll_time * 2 + 5

def get_value_position(x, values):
    for index, value in enumerate(values):
        if value == x:
            return index
    return -1

def check_acc_values_are_increasing(args):
    global VERDICT, OUTPUT
    values = []
    print "Press ENTER to start the record"
    while (True):
        input_line = raw_input("")
        if input_line == "":
            events = poll_for_events(sensor_id, 15, timeout, poll_time)
            if len(events) > 0:
                break
            else:
                OUTPUT = "No data results from sensor poll"
                print_test_result(False, OUTPUT)
                VERDICT = FAILURE
                return
    for event in events:
        values.append(event['data'][axis])
    maxim = max(values)
    max_position = get_value_position(maxim, values)
    if max_position > 3 and are_values_ordered(values[:max_position + 1]) and \
        maxim - values[0] > 0.5 * G:
        message = "Accelerometer values on {} axis were increasing".format(axis)
        print_test_result(True, message)
        VERDICT = SUCCESS
    else:
        values = eliminate_succesive_duplicates(values)
        message = "It was expected an increasing of the values on the {} axis with at least 0.5 * G.".format(axis)
        message += " Returned values were: {}".format(values)
        print_test_result(False, message)
        VERDICT = FAILURE
    OUTPUT = message
# test execution
VERDICT = FAILURE
OUTPUT = ""
PARAMETERS = TC_PARAMETERS('PARAMETERS')
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
axis = PARAMETERS
message = ""
if axis == 'x':
    message = "on the left side(so it moves to the right). "
elif axis == 'y':
    message = "on the bottom(so it moves away from you). "
elif axis == 'z':
    message = "towards the sky. "

menu_items = ["Press enter when you are ready to begin the test",
              {"Push the device " + message: check_acc_values_are_increasing},
              {"Do you want repeat the test (y/n)": validate_result}]

menu(menu_items, menu_items[1:])
