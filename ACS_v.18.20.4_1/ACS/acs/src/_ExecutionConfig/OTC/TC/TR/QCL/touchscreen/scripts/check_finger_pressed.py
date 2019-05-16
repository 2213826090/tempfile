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
events_structure = []

def get_events(arguments):
    global events_structure
    events_structure = touch_util.poll_touchscreen(events=1000, timeout=10)

def check_results(arguments):
    global VERDICT
    global OUTPUT

    if not events_structure:
        VERDICT = FAILURE
        OUTPUT = "Failure. No touch events are retrieved"
        return

    if len(events_structure.action) != 1:
        fail_message = "Failure. There were multiple events detected on the touchscreen. " \
                       "Events obtained: {}".format([x.encode('UTF8') for x in events_structure.action])
        print_to_console_and_logcat(fail_message)
        VERDICT = FAILURE
        OUTPUT = fail_message
        return
    if events_structure.action[0] != "down":
        fail_message = "Failure. Expected to find a finger press event (action down), " \
                       "instead found action '{}'".format(events_structure.action[0])
        print_to_console_and_logcat(fail_message)
        VERDICT = FAILURE
        OUTPUT = fail_message
        return
    print_to_console_and_logcat("Success")
    VERDICT = SUCCESS
    OUTPUT = "Success"

menu_items = ["Press enter when you are ready to begin the test",
              {"Press the touchscreen area with one finger and hold it down without moving it"
               " until further instructions": get_events},
              {"Take the finger away from the touchscreen": check_results},
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
