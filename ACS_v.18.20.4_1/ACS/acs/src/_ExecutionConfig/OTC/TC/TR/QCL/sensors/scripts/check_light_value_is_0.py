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
from util import eliminate_succesive_duplicates
from util import print_test_result
from util import exit_test

def check_light_value_is_0(args):
    global VERDICT, OUTPUT
    values = []
    poll_time = 3
    timeout = poll_time * 2 + 5
    print "Press ENTER to start the record"
    while (True):
        input_line = raw_input("")
        if input_line == "":
            events = poll_for_events(sensor_id, 0, timeout, poll_time)
            break
    is_passed = True
    for event in events:
        values.append(event['data']['light'])
        if event['data']['light'] != 0.0:
            is_passed = False
            break
    if is_passed:
        message  = "All light values were 0"
        print_test_result(True, OUTPUT)
        VERDICT = SUCCESS
    else:
        message = "It was expected that the sensor would record only 0 values. " +\
            "Returned values were: {}".format(values)
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
except:
    OUTPUT = "Sensor was not found"
    print_test_result(False, OUTPUT)
    exit_test()

menu_items = ["Press enter when you are ready to begin the test",
              {"Put the device in a no light environment": check_light_value_is_0},
              {"Do you want repeat the test (y/n)": validate_result}]

menu(menu_items, menu_items[1:])
