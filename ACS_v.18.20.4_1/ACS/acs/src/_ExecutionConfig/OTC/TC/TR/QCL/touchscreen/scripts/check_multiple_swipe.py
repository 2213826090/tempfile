"""
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
"""

import os
import numpy
import touch_util
from util import validate_result
from util import menu
from util import print_to_console_and_logcat
from util import are_values_ordered
from util import exit_test

VERDICT = FAILURE
swipe_direction = TC_PARAMETERS('swipe_direction')

expected_directions = ["up", "down", "left", "right"]
if swipe_direction not in expected_directions:
    print_to_console_and_logcat("Incorrect value '{}' for parameter swipe_direction. \n".format(swipe_direction) +
                                "It should be one of: " + expected_directions)
    exit_test()
press_message = "Using five fingers, swipe {} on the touchscreen.".format(swipe_direction)

def are_values_constant(values, rel_std_dev=0.05):
    rsd = numpy.std(values) / numpy.mean(values)
    if rsd <= rel_std_dev:
        return True
    else:
        print_to_console_and_logcat("Values are not relative constant. Found relative standard deviation: {:.1f}%, "
                                    "but we expect it to be lower than {:.1f}%".format(rsd*100, rel_std_dev*100))
        return False

def check_multiple_swipe(arguments):
    global VERDICT
    global OUTPUT

    events_structure = touch_util.poll_touchscreen(events=1000, timeout=10, stop_finger_up=True)
    if not events_structure:
        VERDICT = FAILURE
        OUTPUT = "Failure. No touch events are retrieved"
        return

    x_coords = []
    y_coords = []
    for index, fingers_nr in enumerate(events_structure.fingers_number):
        # when at least one finger is lifted we won't take into consideration other events
        if events_structure.action[index] == "pointer up":
            break
        for finger in range(0, fingers_nr):
            if len(x_coords) <= finger:
                x_coords.append([])
            x_coords[finger].append(events_structure.x[index][finger])
            if len(y_coords) <= finger:
                y_coords.append([])
            y_coords[finger].append(events_structure.y[index][finger])

    # x_coords and y_coords are the same length
    fingers_nr = len(x_coords)
    if fingers_nr < 5:
        print_to_console_and_logcat("Failure. Touch should be pressed with at least 5 fingers")
        print_to_console_and_logcat("Failure. There were recorded only " + str(fingers_nr) + " fingers")
        VERDICT = FAILURE
        OUTPUT = "Failure. Only " + str(fingers_nr) + " fingers detected"

    else:
        passed = True
        for finger in range(0, fingers_nr):
            if swipe_direction == "down":
                constant = "x coords"
                ordered = "y coords"
                order = 1
            elif swipe_direction == "up":
                constant = "x coords"
                ordered = "y coords"
                order = -1
            elif swipe_direction == "right":
                constant = "y coords"
                ordered = "x coords"
                order = 1
            elif swipe_direction == "left":
                constant = "y coords"
                ordered = "x coords"
                order = -1

            if constant == "x coords":
                constant_vals = x_coords[finger]
            else:
                constant_vals = y_coords[finger]

            if ordered == "x coords":
                ordered_vals = x_coords[finger]
            else:
                ordered_vals = y_coords[finger]

            if not are_values_ordered(ordered_vals, order, 0.99):
                passed = False
                fail_message = "Failure. For at least one finger, values for {} did not increase " \
                               "as they should.".format(ordered)
                print_to_console_and_logcat(fail_message)
                VERDICT = FAILURE
                OUTPUT = fail_message
                break
            elif not are_values_constant(constant_vals, 0.2):
                passed = False
                fail_message = "Failure. For at least one finger, values for {} are not relative constant " \
                               "as they should be.".format(finger+1, constant)
                print_to_console_and_logcat(fail_message)
                VERDICT = FAILURE
                OUTPUT = fail_message
                break
        if passed:
            print_to_console_and_logcat("Success")
            VERDICT = SUCCESS
            OUTPUT = "Success"

menu_items = ["Press enter when you are ready to begin the test",
              {press_message: check_multiple_swipe},
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
