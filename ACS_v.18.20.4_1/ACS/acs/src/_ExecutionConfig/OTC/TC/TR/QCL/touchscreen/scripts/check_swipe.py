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

import touch_util
from util import validate_result
from util import menu
from util import print_to_console_and_logcat
from util import are_values_ordered

VERDICT = FAILURE
DIAGONAL = TC_PARAMETERS('DIAGONAL')
swipe_message = ""
if DIAGONAL == "primary":
    swipe_message = "Swipe with one finger from the top left corner of the touchscreen to the bottom right"
elif DIAGONAL == "secondary":
    swipe_message = "Swipe with one finger from the top right corner of the touchscreen to the bottom left"

def check_axis(arguments):
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

    if DIAGONAL == "primary":
        if not (touch_util.is_coord_in_corner(x_coords[0], y_coords[0], "upper-left") and
                    touch_util.is_coord_in_corner(x_coords[-1], y_coords[-1], "bottom-right")):
            print_to_console_and_logcat("Failure. Swipe should start from the top left corner "
                                        "and end in the bottom right corner")
            VERDICT = FAILURE
            OUTPUT = "Failure. Swipe start coordinate " + str(x_coords[0]) +" "+ str(y_coords[0]) + " and end cooridnates " + \
                     str(x_coords[-1]) +" "+ str(y_coords[-1]) + " don't match the intended swipe gesture"
            return

        for coords, coords_name, axis in zip([x_coords,y_coords],["x_coords", "y_coords"], ["X", "Y"]):
            if not are_values_ordered(coords):
                print_to_console_and_logcat("Failure. Events coordinates should increase on {} axis.".format(axis))
                VERDICT = FAILURE
                OUTPUT = "Failure. Events coordinates should increase on {} axis.".format(axis)
                return

        print_to_console_and_logcat("Success")
        VERDICT = SUCCESS
        OUTPUT = "Success"

    if DIAGONAL == "secondary":
        if not (touch_util.is_coord_in_corner(x_coords[0], y_coords[0], "upper-right") and
                    touch_util.is_coord_in_corner(x_coords[-1], y_coords[-1], "bottom-left")):
            print_to_console_and_logcat("Failure. Swipe should start from the top right corner "
                                        "and end in the bottom left corner")
            VERDICT = FAILURE
            OUTPUT = "Failure. Swipe start coordinate " + str(x_coords[0]) +" "+ str(y_coords[0]) + " and end cooridnates " + \
                     str(x_coords[-1]) +" "+ str(y_coords[-1]) + " don't match the intended swipe gesture"
            return
        if not are_values_ordered(x_coords, -1):
            print_to_console_and_logcat("Failure. Events coordinates should decrease on X axis.")
            VERDICT = FAILURE
            OUTPUT = "Failure. Events along x axis are not in decrease order"
            return
        if not are_values_ordered(y_coords):
            print_to_console_and_logcat("Failure. Events coordinates should increase on Y axis.")
            VERDICT = FAILURE
            OUTPUT = "Failure. Events along y axis are not in decrease order"
            return
        print_to_console_and_logcat("Success")
        VERDICT = SUCCESS
        OUTPUT = "Success"

menu_items = ["Press enter when you are ready to begin the test",
              {swipe_message: check_axis},
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
