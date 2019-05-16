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

VERDICT = FAILURE
MULTIPLE_FINGERS = TC_PARAMETERS('MULTIPLE_FINGERS')
press_message = ""
if MULTIPLE_FINGERS == "True":
    press_message = "Press with the fingertip of five of your fingers on the touchscreen\n" + \
                    "Then, for several seconds, gradually increase the fingerprints size on the touchscreen."
elif MULTIPLE_FINGERS == "False":
    press_message = "Press with the fingertip of one finger on the touchscreen\n" + \
                    "Then, for several seconds, gradually increase the fingerprint size on the touchscreen."
else:
    print_to_console_and_logcat("Incorrect value for MULTIPLE_FINGERS parameter. " + \
                                "It should be True or False")
    exit_test()

def check_size(arguments):
    global VERDICT
    global OUTPUT

    events_structure = touch_util.poll_touchscreen(events=1000, timeout=10, stop_finger_up=True)
    if not events_structure:
        VERDICT = FAILURE
        OUTPUT = "Failure. No touch events are retrieved"
        return

    size_values = []
    for index, fingers_nr in enumerate(events_structure.fingers_number):
        for finger in range(0, fingers_nr):
            if len(size_values) <= finger:
                size_values.append([])
            size_values[finger].append(events_structure.size[index][finger])
    if MULTIPLE_FINGERS == "False":
        if are_values_ordered(size_values[0]):
            print_to_console_and_logcat("Success")
            VERDICT = SUCCESS
            OUTPUT = "Success"
        else:
            print_to_console_and_logcat("Failure. Finger size should increase.")
            VERDICT = FAILURE
            OUTPUT = "Failure. Finger size didn't increase."
    else:
        fingers_nr = len(size_values)
        if fingers_nr < 5:
            print_to_console_and_logcat("Failure. Touch should be pressed with at least 5 fingers")
            print_to_console_and_logcat("Failure. There were recorded only " + str(fingers_nr) + \
                                        " fingers")
            VERDICT = FAILURE
            OUTPUT = "Failure. There were recorded only " + str(fingers_nr) + \
                                        " fingers"
        else:
            passed = True
            for finger in range(0, fingers_nr):
                if not are_values_ordered(size_values[finger]):
                    passed = False
                    print_to_console_and_logcat("Failure. Fingers size should increase")
                    VERDICT = FAILURE
                    OUTPUT = "Failure. Finger size didn't increase."
                    break
            if passed:
                print_to_console_and_logcat("Success")
                VERDICT = SUCCESS
                OUTPUT = "Success"

menu_items = ["Press enter when you are ready to begin the test",
              {press_message: check_size},
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
