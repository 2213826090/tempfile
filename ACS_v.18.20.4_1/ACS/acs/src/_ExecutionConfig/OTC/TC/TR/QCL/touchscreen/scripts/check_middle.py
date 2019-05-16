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

import touch_util
from util import validate_result
from util import menu
from util import print_to_console_and_logcat

VERDICT = FAILURE

def check_axis(arguments):
    global VERDICT
    global OUTPUT
    x_max, x_min, y_max, y_min = touch_util.retrieve_resolution_geteven()

    events_structure = touch_util.poll_touchscreen(events=1, timeout=20)
    if not events_structure:
        VERDICT = FAILURE
        OUTPUT = "Failure. No touch events are retrieved"
        return

    x_middle = (x_min + x_max) / 2
    y_middle = (y_min + y_max) / 2
    if abs(events_structure.x[0][0] - x_middle) < 50 and abs(events_structure.y[0][0] - y_middle) < 50:
        print_to_console_and_logcat("Success")
        VERDICT = SUCCESS
        OUTPUT = "Success"
    else:
        print_to_console_and_logcat("Middle of the touchscreen is located at coordinate [{} {}].".format(x_middle, y_middle))
        print_to_console_and_logcat("Touch event was found at coordinate [{} {}]".format(events_structure.x[0][0], events_structure.y[0][0]))
        print_to_console_and_logcat("Failure. Touch event should be near the middle of the touchscreen")
        VERDICT = FAILURE
        OUTPUT = "Failure. Coordinates don't match middle of the screen. Expected: " + str(x_middle) + " " + str(y_middle) +\
                 " Received: " + str(events_structure.x[0][0]) + " " +str(events_structure.y[0][0])

menu_items = ["Press enter when you are ready to begin the test",
              {"Press on the middle of the touchscreen": check_axis},
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
