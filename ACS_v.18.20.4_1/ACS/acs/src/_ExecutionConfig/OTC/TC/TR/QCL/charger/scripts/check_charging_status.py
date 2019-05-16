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
from charger_util import wait_for_event
from charger_util import get_charger_info
from charger_util import get_charging_status
from util import validate_result
from util import menu
from util import print_to_console_and_logcat
from util import exit_test

VERDICT = FAILURE
OUTPUT = ""
check_option = TC_PARAMETERS('check_option')
COMP_CONF = TC_PARAMETERS('COMP_CONF')
get_charger_info(COMP_CONF)

if check_option == "connect":
    pre_check_mess = "Make sure the power supply is disconnected and battery is not fully charged"
    pre_check_status = "Discharging"
    check_mess = "Connect the power supply"
    check_status = "Charging"
elif check_option == "disconnect":
    pre_check_mess = "Make sure the power supply is connected"
    pre_check_status = ["Charging", "Full"]
    check_mess = "Disconnect the power supply"
    check_status = "Discharging"
else:
    OUTPUT =  "Invalid value for parameter 'check_option'!!!'"
    print_to_console_and_logcat(OUTPUT)
    exit_test()

def wait_for_status(args):
    global VERDICT, OUTPUT
    if wait_for_event("power status", check_status):
        message = "SUCCESS"
        print_to_console_and_logcat(message)
        VERDICT = SUCCESS
    else:
        message = "FAILURE: Did not find power status '{}', instead found status '{}'"
        message = message.format(check_status,get_charging_status())
        print_to_console_and_logcat(message)
        VERDICT = FAILURE
    OUTPUT = message

def pre_check_verification(args):

    global VERDICT, OUTPUT
    if wait_for_event("power status", pre_check_status):
        menu(validation_menu, initial_menu_items)
    else:
        OUTPUT = "FAILURE: Did not find power status '{}'".format(pre_check_status)
        print_to_console_and_logcat(OUTPUT)
        VERDICT = FAILURE
        menu(validation_menu[1:], initial_menu_items)

initial_menu_items = ["Press enter when you are ready to begin the test",
                      {pre_check_mess: pre_check_verification}]

validation_menu = [{check_mess: wait_for_status},
                   {"Do you want to repeat the test ? (y/n)": validate_result}]

menu(initial_menu_items, initial_menu_items)
