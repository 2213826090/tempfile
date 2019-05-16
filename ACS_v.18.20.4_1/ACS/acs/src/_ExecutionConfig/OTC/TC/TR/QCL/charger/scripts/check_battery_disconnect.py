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

from charger_util import wait_for_event
from charger_util import get_charger_info
from util import validate_result
from util import menu
from util import print_to_console_and_logcat

VERDICT = FAILURE
OUTPUT = ""
COMP_CONF = TC_PARAMETERS('COMP_CONF')
get_charger_info(COMP_CONF)

def wait_for_battery_disconnect(args):
    global VERDICT, OUTPUT
    if wait_for_event("battery presence", '0'):
        OUTPUT = "SUCCESS"
        print_to_console_and_logcat(OUTPUT)
        VERDICT = SUCCESS
    else:
        OUTPUT = "FAILURE: Did not find battery disconnected"
        print_to_console_and_logcat(OUTPUT)
        VERDICT = FAILURE

def wait_for_power_and_battery(args):
    global VERDICT, OUTPUT
    if wait_for_event("power status", ["Charging", "Full"]) and wait_for_event("battery presence", '1'):
        menu(validation_menu, initial_menu_items)
    else:
        OUTPUT = "FAILURE: Did not find power status 'Charging' or 'Full' and battery connected"
        print_to_console_and_logcat(OUTPUT)
        VERDICT = FAILURE
        menu(validation_menu[1:], initial_menu_items)

initial_menu_items = ["Press enter when you are ready to begin the test",
                      {"Make sure the power supply and battery are connected": wait_for_power_and_battery}]

validation_menu = [{"Disconnect the battery": wait_for_battery_disconnect},
                   {"Do you want to repeat the test ? (y/n)": validate_result}]

menu(initial_menu_items, initial_menu_items)
