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
import numpy
from sensors_util import get_sensor_hal_info
from sensors_util import poll_for_events
from util import print_test_result
from util import validate_result
from util import menu
from util import exit_test

poll_time = 3
G = 9.8
acc_error = 1
acc_sd = 0.5
timeout = poll_time * 2 + 5
axes = ['x','y','z']

def check_values(axis_avg, axis_sd, expected_avg):
    for axis in axes:
        if abs(expected_avg[axis] - axis_avg[axis]) > acc_error:
            return False
        if axis_sd[axis] > acc_sd:
            return False
    return True

def check_acc_coords(args):
    global VERDICT, OUTPUT
    acc_values = {}
    for axis in axes:
        acc_values[axis] = []
    print "Press ENTER to start the record"
    while (True):
        input_line = raw_input("")
        if input_line == "":
            events = poll_for_events(sensor_id, 0, timeout, poll_time)
            if len(events) > 0:
                break
            else:
                OUTPUT = "No data results from sensor poll"
                print_test_result(False, OUTPUT)
                VERDICT = FAILURE
                return
    for event in events:
        for axis in axes:
            acc_values[axis].append(event['data'][axis])
    expected_avg = {}
    axis_avg = {}
    axis_sd = {}
    for axis in axes:
        expected_avg[axis] = 0.0
        axis_avg[axis] = numpy.mean(acc_values[axis])
        axis_sd[axis] = numpy.std(acc_values[axis])
    expected_avg[AXIS_ORIENTATION] = G
    message = ""
    axis_message = "{} axis expected value: {}, , obtained average: {}  standard deviation: {}\n"
    for axis in axes:
        message +=  axis_message.format(axis, expected_avg[axis], axis_avg[axis], axis_sd[axis])
    if check_values(axis_avg, axis_sd, expected_avg):
        print_test_result(True, message)
        VERDICT = SUCCESS
    else:
        print_test_result(False, message)
        VERDICT = FAILURE
    OUTPUT = message

# test execution
VERDICT = FAILURE
OUTPUT = ""
AXIS_ORIENTATION = TC_PARAMETERS('PARAMETERS')
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
if AXIS_ORIENTATION == 'x':
    message = "Place the device on a flat horizontal surface with the device's screen " + \
              "perpendicular to the ground, having its second orientation"
elif AXIS_ORIENTATION == 'y':
    message = "Place the device on a flat horizontal surface with the device's screen " + \
              "perpendicular to the ground, having its default orientation "
elif AXIS_ORIENTATION == 'z':
    message = "Place the device on a flat horizontal surface with the device's screen " + \
              "pointing up"
menu_items = ["Press enter when you are ready to begin the test",
              {message: check_acc_coords},
              {"Do you want repeat the test (y/n)": validate_result}]

menu(menu_items,menu_items[1:])
