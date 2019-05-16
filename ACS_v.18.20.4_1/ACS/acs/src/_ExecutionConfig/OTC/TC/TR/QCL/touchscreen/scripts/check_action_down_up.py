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
from util import print_to_console_and_logcat
from util import validate_result
from util import menu
VERDICT = FAILURE

def check_action(arguments):
    global VERDICT
    global OUTPUT

    events_structure = touch_util.poll_touchscreen(events=2, timeout=20)
    if not events_structure:
        VERDICT = FAILURE
        OUTPUT = "Failure. No touch events are retrieved"
        return

    if str(events_structure.action[0]) == "down" and str(events_structure.action[1]) == "up":
        print_to_console_and_logcat("Success")
        VERDICT = SUCCESS
        OUTPUT = "Success"
    else:
        fail_message = "Failure. First 2 action events obtained are: " + str(events_structure.action[0]) + " " + \
                       str(events_structure.action[1])
        print_to_console_and_logcat(fail_message)
        VERDICT = FAILURE
        OUTPUT = fail_message

menu_items = ["Press enter when you are ready to begin the test",
              {"Press with one finger on the touchscreen": check_action},
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
