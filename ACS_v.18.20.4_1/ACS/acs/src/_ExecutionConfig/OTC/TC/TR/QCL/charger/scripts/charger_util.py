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

import subprocess
import os
import sys
import time
import json
import difflib

# Adding location of util.py to the sys.path in order to import it in the testcase file
# Note that you must import 'util' module after importing this current module
util_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if util_path not in sys.path:
    sys.path.append(util_path)
from util import print_to_console_and_logcat
from util import exit_test

driver_name = ""

def get_property(prop):
    global driver_name
    location = '/sys/class/power_supply/'

    proc = subprocess.Popen(['adb', 'shell', 'ls ' + location], stdout=subprocess.PIPE)
    available_drivers = proc.stdout.read().split()
    if not available_drivers:
        print "Did not find any files in '{}' on the android device".format(location)
        exit_test()

    # The driver name is not necessarily equal to the driver filename found in the .desc file
    if driver_name not in available_drivers:
        if driver_name.replace('_', '-') in available_drivers:
            driver_name = driver_name.replace('_', '-')
        else:
            max_ratio = -1
            matched_item = None
            for found_driver in available_drivers:
                match_ratio = difflib.SequenceMatcher(None, driver_name, found_driver).ratio()
                if match_ratio > max_ratio:
                    max_ratio = match_ratio
                    matched_item = found_driver
                # if name is similar or it is the only available power_supply driver, we select it
                if max_ratio > 0.6 or len(available_drivers) == 1:
                    driver_name = matched_item
                else:
                    print "Found available drivers: {}".format(available_drivers)
                    print "Did not find driver with similar name with '{}'".format(driver_name)
                    print "Warning: Choosing driver '{}' but it may not be the correct one".format(matched_item,
                                                                                                   driver_name)
                    driver_name = matched_item

    prop_file = os.path.join(location, driver_name, prop)

    proc = subprocess.Popen(['adb', 'shell', 'cat ' + prop_file], stdout=subprocess.PIPE)
    output = proc.stdout.read().strip("\r\n")

    if "No such file or directory" in output:
        print "Did not find file '{}' present on the android device".format(prop_file)
        exit_test()

    return output

def get_charging_status():
    return get_property('status')

def get_battery_presence():
    return get_property('present')

def get_constant_charge_current():
    return get_property('constant_charge_current')

def get_online_status():
    return get_property('online')

def wait_for_event(event, expected_state, timeout=20):
    if event == "battery presence":
        get_method = get_battery_presence
    elif event == "power status":
        get_method = get_charging_status
    elif event == "charger online":
        get_method = get_online_status
    else:
        print "Unknown event: '{}'!!!".format(event)
        exit_test()
    start = time.time()
    while time.time() - start < timeout:
        found_state = get_method()
        if isinstance(expected_state, basestring):
            comparison = found_state == expected_state
        elif isinstance(expected_state, list):
            comparison = found_state in expected_state
        else:
            comparison = False
        if comparison:
            print_to_console_and_logcat("Found {}: '{}'".format(event, found_state))
            return True
    print_to_console_and_logcat("Timeout waiting for '{}' to be '{}'".format(event, expected_state))
    return False

def get_charger_info(info_string):
    component_json = {}
    global driver_name
    info_string = info_string.replace("\"", "")
    info_string = info_string.replace("\'", "\"")
    try:
        component_json = json.loads(info_string)
        driver_name = component_json['driver']['name']
    except:
        print ("Failed to get charger info!!!")
        exit_test()
    return component_json