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
from util import validate_result
from util import menu
from util import are_values_ordered
from util import print_test_result
from util import exit_test

def check_temperature_values_are_ordered(args):
    global VERDICT, OUTPUT
    values = []
    poll_time = 5
    timeout = poll_time * 2 + 5
    p = 0.95
    if order == -1:
        p = 1.05
    print "Press ENTER to start the record"
    while (True):
        input_line = raw_input("")
        if input_line == "":
            events = poll_for_events(sensor_id, 0, timeout, poll_time)
            break
    for event in events:
        values.append(event['data']['temperature'])
    if are_values_ordered(values, order):
        message = "Temperature value " + order_s + "d"
        print_test_result(True, message)
        VERDICT = SUCCESS
    else:
        message = "Sensor values were not ordered"
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
order = int(PARAMETERS)
if order == 1:
    order_s = "increase"
else:
    order_s = "decrease"
menu_items = ["Press enter when you are ready to begin the test",
              {"Put a temperature source in front of the device and gradually " \
               + order_s + " the temperature value": check_temperature_values_are_ordered},
              {"Do you want repeat the test (y/n)": validate_result}]

menu(menu_items, menu_items[1:])
