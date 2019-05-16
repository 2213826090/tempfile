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
from util import menu_next_item
from util import print_to_console_and_logcat
from util import exit_test

VERDICT = FAILURE
OUTPUT = ""
COMP_CONF = TC_PARAMETERS('COMP_CONF')
charger_info = get_charger_info(COMP_CONF)
try:
    transition_count = int(TC_PARAMETERS('Transition_count'))
except:
    OUTPUT = "Parameter 'Transition_count' should be an integer!'"
    print_to_console_and_logcat(OUTPUT)
    exit_test()
if transition_count < 1:
    OUTPUT = "Parameter 'Transition_count' should be greater than 1!'")
    print_to_console_and_logcat(OUTPUT)
    exit_test()

def wait_for_status():
    global VERDICT, OUTPUT
    global current_iteration
    tmp_status = check_status
    if check_status == "Charging":
        tmp_status = [check_status, "Full"]
    if wait_for_event("power status", tmp_status):
        return True
    else:
        OUTPUT = "FAILURE: Did not find power status '{}'".format(check_status)
        print_to_console_and_logcat(OUTPUT)
        VERDICT = FAILURE
        print "Do you want to repeat the test ? (y/n)"
        while True:
            input_line = raw_input("")
            if input_line == "y":
                current_iteration = 0
                return True
            elif input_line == "n":
                return False

current_iteration = 0

while current_iteration < transition_count:
    current_iteration += 1
    if current_iteration % 2 == 0:
        check_mess = "Connect the power supply"
        check_status = "Charging"
    else:
        check_mess = "Disconnect the power supply"
        check_status = "Discharging"

    if current_iteration == 1:
        print "Press enter when you are ready to begin the test"
        menu_next_item()
        pass

    print "Test iteration #{}".format(current_iteration)
    print check_mess

    if not wait_for_status():
        break

    if current_iteration == transition_count:
        OUTPUT = "SUCCESS"
        print_to_console_and_logcat(OUTPUT)
        VERDICT = SUCCESS
        break