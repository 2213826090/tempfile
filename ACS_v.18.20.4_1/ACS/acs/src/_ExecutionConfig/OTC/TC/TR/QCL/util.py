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
import shlex
import numpy
import sys, os
from itertools import groupby

def print_to_console_and_logcat(message, tag="TestTag"):
    '''
    :param message: The message that is going to be printed to logcat and console.
                    Note that '\n' will be transformed in ' ' in the logcat
    :param tag: The tag associated to the message
    :return:
    '''
    adb_cmd = 'adb '
    adb_arg = 'shell log -p i -t '
    max_len = len(adb_cmd) + 1024
    message_split = message.split()

    while len(message_split) > 0:
        cmd = adb_cmd + adb_arg + tag
        while len(message_split) > 0 and len(cmd) + len(message_split[0]) + 1 <= max_len:
            cmd += " " + message_split.pop(0)
        cmd = cmd.replace("'","\\'")
        proc = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE)
        proc.wait()
    print "{}: {}".format(tag, message)

def eliminate_succesive_duplicates(values):
    return [x[0] for x in groupby(values)]

def are_values_ordered(values, order=1, tolerance=0.95):
    values = eliminate_succesive_duplicates(values)
    norm = abs(min(values)) + 1.00
    max_num_of_printed_vals = 6
    if order == 1:
        order_name = "ascending"
    else:
        order_name = "descending"
    values = [x + norm for x in values]
    for index, value in enumerate(values):
        if index > 0 and pow(value / values[index - 1], order) < tolerance:
            unordered_vals = []
            for i in range(-max_num_of_printed_vals/2, max_num_of_printed_vals/2):
                if 0 <= index + i < len(values):
                    unordered_vals.append(round(values[index + i] - norm, 2))
            print_to_console_and_logcat("Values are not in {} order. For example the following sample values: {}"
                                        .format(order_name, unordered_vals))
            return False
    if pow(values[-1] / values[0], order) <= 1:
        print_to_console_and_logcat("Values are not in {} order. The first value is '{}' and the last value is '{}'"
                                    .format(order_name, values[0]-norm, values[-1]-norm))
        return False
    else:
        return True

def print_test_result(test_passed, message):
    if test_passed:
        test_result = "PASSED"
    else:
        test_result = "FAILED"
    test_result = test_result + " " + message
    print_to_console_and_logcat(test_result)

def validate_result(list_menu_items):
    while (True):
        input_line = raw_input("")
        if input_line == "y":
            menu(list_menu_items, list_menu_items)
            break
        elif input_line == "n":
            break

def menu_next_item():
    while (True):
        input_line = raw_input("")
        if input_line == "":
            break

def menu(menu_steps, args):
    for menu_entry in menu_steps:
        if isinstance(menu_entry, basestring):
            print menu_entry
            menu_next_item()
        if isinstance(menu_entry, dict):
            print menu_entry.keys()[0]
            menu_entry.values()[0](args)

########################################################################
# Decide if a given set of measurements has the expected frequency based
# on the mean and relative standard deviation
#
# values_array is an array containing the time difference between
# consecutive samples, in seconds
#
# desired_frequency is the tested frequency
########################################################################
def is_frequency_in_expected_range(desired_frequency, values_array, greater=False,
                                   avg_tolerance=0.1, max_rel_st_dev=0.18):
    desired_period = 1.0 / desired_frequency
    average = numpy.mean(values_array)
    freq_average = 1.0 / average
    standard_deviation = numpy.std(values_array)
    relative_standard_deviation = standard_deviation / average
    result = {}
    if greater:
        result["message"] = "minimum "
    else:
        result["message"] = ""
    result["message"] += "desired_period {}, obtained average {}, sd {:.2f} \n".\
        format(desired_period, average, relative_standard_deviation)
    if greater:
        result["message"] += "minimum "
    result["message"] += "desired frequency {:.2f}, obtained frequency average {:.2f}".\
        format(desired_frequency, freq_average)
    max_avg = (1 + avg_tolerance) * desired_period
    min_avg = (1 - avg_tolerance) * desired_period
    if average < max_avg and (greater or min_avg < average) and \
                    relative_standard_deviation < max_rel_st_dev:
        result["test_passed"] = True
    else:
        result["test_passed"] = False
    return result

def exit_test():
    sys.stdout.flush()
    sys.exit()

