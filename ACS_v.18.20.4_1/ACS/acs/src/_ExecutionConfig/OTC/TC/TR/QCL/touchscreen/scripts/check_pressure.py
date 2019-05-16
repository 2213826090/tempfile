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
MULTIPLE_FINGERS = TC_PARAMETERS('MULTIPLE_FINGERS')
press_message = ""
if MULTIPLE_FINGERS == "True":
    press_message = "Press with the fingertip of five of your fingers on the touchscreen"
elif MULTIPLE_FINGERS == "False":
    press_message = "Press with the fingertip of one finger on the touchscreen"

def check_pressure(arguments):
    global VERDICT
    global OUTPUT

    events_structure = touch_util.poll_touchscreen(events=1000, timeout=10, stop_finger_up=True)
    if not events_structure:
        VERDICT = FAILURE
        OUTPUT = "Failure. No touch events are retrieved"
        return

    pressure_values = []
    for index, fingers_nr in enumerate(events_structure.fingers_number):
        for finger in range(0, fingers_nr):
            if len(pressure_values) <= finger:
                pressure_values.append([])
            pressure_values[finger].append(events_structure.pressure[index][finger])
    if MULTIPLE_FINGERS == "False":
        if not 0 in pressure_values[0]:
            print_to_console_and_logcat("Success")
            VERDICT = SUCCESS
            OUTPUT = "Success"
        else:
            fail_message = "Failure. Found a value of zero for pressure for the finger. " \
                           "Pressure values should be non-zero."
            print_to_console_and_logcat(fail_message)
            VERDICT = FAILURE
            OUTPUT = fail_message
    else:
        fingers_nr = len(pressure_values)
        if fingers_nr < 5:
            print_to_console_and_logcat("Failure. Touch should be pressed with at least 5 fingers")
            print_to_console_and_logcat("Failure. It were recorded only " + str(fingers_nr) + \
                                        " fingers")
            VERDICT = FAILURE
            OUTPUT = "Failure. Only "+ str(fingers_nr) + " fingers detected"
        else:
            passed = True
            for finger in range(0, fingers_nr):
                if 0 in pressure_values[finger]:
                    passed = False
                    fail_message = "Failure. Found a value of zero for pressure for one of the fingers. " \
                                   "Pressure values should be non-zero."
                    print_to_console_and_logcat(fail_message)
                    VERDICT = FAILURE
                    OUTPUT = fail_message
                    break
            if passed:
                print_to_console_and_logcat("Success")
                VERDICT = SUCCESS
                OUTPUT = "Success"

menu_items = ["Press enter when you are ready to begin the test",
              {press_message: check_pressure},
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
