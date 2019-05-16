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
import touch_util
from util import validate_result
from util import menu
from util import print_to_console_and_logcat
from util import are_values_ordered
from util import exit_test

EDGE = TC_PARAMETERS('EDGE')
VERDICT = FAILURE

x_max, x_min, y_max, y_min = touch_util.retrieve_resolution_geteven()
print_to_console_and_logcat(
    "Resolution from getevent obtained: " + " x_max: " + str(x_max) + " x_min: " + \
    str(x_min) + " y_max: " + str(y_max) + " y_min: " + str(y_min))

swipe_message = ""
if EDGE == "top":
    swipe_message = "Swipe with one finger from the top left corner of the touchscreen to the top right"
elif EDGE == "bottom":
    swipe_message = "Swipe with one finger from the bottom left corner of the touchscreen to the bottom right"
elif EDGE == "left":
    swipe_message = "Swipe with one finger from the top left corner of the touchscreen to the bottom left"
elif EDGE == "right":
    swipe_message = "Swipe with one finger from the top right corner of the touchscreen to the bottom right"
elif EDGE == "non_touch":
    swipe_message = "Swipe with one finger from the touchscreen area to a non-touchscreen area"
else:
    print_to_console_and_logcat("Edge parameter is incorrect")
    exit_test()

def are_values_correct(x_coords, y_coords):
    if min(x_coords) < x_min or max(x_coords) > x_max:
        print_to_console_and_logcat("Failure. Some of the values excedeed the maximum or "
                                    "minimum values for ABS_X (min:{} max:{})".format(x_min, x_max))
        print_to_console_and_logcat("Found x_coords min:{} max:{}".format(min(x_coords), max(x_coords)))
        return False

    if min(y_coords) < y_min or max(y_coords) > y_max:
        print_to_console_and_logcat("Failure. Some of the values excedeed the maximum or "
                                    "minimum values for ABS_Y (min:{} max:{})".format(y_min, y_max))
        print_to_console_and_logcat("Found y_coords min:{} max: {}".format(min(y_coords), max(y_coords)))
        return False

    for coords, coords_name in zip([x_coords, y_coords], ["x_coords", "y_coords"]):
        if min(coords) < 0:
            print_to_console_and_logcat("Failure. All coordinates should be positive")
            print_to_console_and_logcat("Failure. Found negative {}: {}".format(coords_name), min(coords))
            return False

    return True

def are_values_on_the_edge(x_coords, y_coords):
    if EDGE == 'left':
        const_axis, ordered_axis = (x_coords, y_coords)
        const_axis_name, ordered_axis_name = ("x axis", "y axis")
        edge_val = x_min
        min_ordered, max_ordered = (y_min, y_max)
    elif EDGE == "right":
        const_axis, ordered_axis = (x_coords, y_coords)
        const_axis_name, ordered_axis_name = ("x axis", "y axis")
        edge_val = x_max
        min_ordered, max_ordered = (y_min, y_max)
    elif EDGE == "top":
        const_axis, ordered_axis = (y_coords, x_coords)
        const_axis_name, ordered_axis_name = ("y axis", "x axis")
        edge_val = y_min
        min_ordered, max_ordered = (x_min, x_max)
    elif EDGE == "bottom":
        const_axis, ordered_axis = (y_coords, x_coords)
        const_axis_name, ordered_axis_name = ("y axis", "x axis")
        edge_val = y_max
        min_ordered, max_ordered = (x_min, x_max)

    pixel_margin = 50
    for const_coord in const_axis:
        if abs(const_coord - edge_val) > pixel_margin:
            print_to_console_and_logcat("Found value {} for {} that is too far from the touchscreen edge ({})"
                                        .format(const_coord, const_axis_name, edge_val))
            return False
    if abs(ordered_axis[0] - min_ordered) > pixel_margin:
        print_to_console_and_logcat("Swipe did not start near the expected corner. Found {} coordinate '{}' "
                                    "which is too far from the corner value ({})".
                                    format(ordered_axis_name, ordered_axis[0], min_ordered))
        return False
    if abs(ordered_axis[-1] - max_ordered) > pixel_margin:
        print_to_console_and_logcat("Swipe did not end near the expected corner. Found {} coordinate '{}' "
                                    "which is too far from the corner value ({})"
                                    .format(ordered_axis_name, ordered_axis[-1], max_ordered))
        return False
    if not are_values_ordered(ordered_axis):
        return False
    return True

def check_edge(arguments):
    global VERDICT
    global OUTPUT

    events_structure = touch_util.poll_touchscreen(events=1000, timeout=10, stop_finger_up=True)
    if not events_structure:
        VERDICT = FAILURE
        OUTPUT = "Failure. No touch events are retrieved"
        return

    x_coords = []
    y_coords = []
    for x_event in events_structure.x:
        x_coords.append(x_event[0])
    for y_event in events_structure.y:
        y_coords.append(y_event[0])

    if are_values_correct(x_coords, y_coords):
        if EDGE == "non_touch":
            print_to_console_and_logcat("Success")
            VERDICT = SUCCESS
            OUTPUT = "Success"
        elif are_values_on_the_edge(x_coords, y_coords):
            print_to_console_and_logcat("Success")
            VERDICT = SUCCESS
            OUTPUT = "Success"
        else:
            fail_message = "Failure. Events coordinates were not appropriate for the " + EDGE + " edge."
            print_to_console_and_logcat(fail_message)
            VERDICT = FAILURE
            OUTPUT = fail_message
    else:
        VERDICT = FAILURE
        OUTPUT = "Failure. X and y values are not correct " + str(x_coords) + str(y_coords)

menu_items = ["Press enter when you are ready to begin the test",
              {swipe_message: check_edge},
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
