"""
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
"""

import subprocess
import time
import numpy
import json
import sys
import os
import threading
import Queue
import touch_util
from util import validate_result
from util import menu
from util import print_to_console_and_logcat
from util import exit_test

touch_timeout = 10  # first touch timeout
minimum_events = 30  # The minimum number of events in order to make a decision
sufficient_events = 100  # The sufficient number of events to make a decision
# The default sleep duration for pm_test is 5s
# Just to be sure that all processes are started wait 20s
max_sleep_duration = 20
# to find out real seconds, we divide by 10^9 (from nanosecond to second)
scale = 1000000000.0
FNULL = open(os.devnull, 'w')

suspend_to_ram = [["adb", "shell", "echo", "core", ">", "/sys/power/pm_test"],
                  ["adb", "shell", "echo", "mem", ">", "/sys/power/state"]]
touch_poll_cmd = ['adb', 'shell', 'touchtest']

def read_process_output(cmd, queue):
    while True:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=FNULL)
        # while the process is running read from stdout
        while proc.poll() is None:
            try:
                for line in iter(proc.stdout.readline, ''):
                    queue.put(line)
                proc.stdout.close()
            except:
                pass

q = Queue.Queue()
t = threading.Thread(target=read_process_output, args=(touch_poll_cmd, q))
t.daemon = True
t.start()

def check_sleep(arguments):
    global VERDICT
    global OUTPUT
    VERDICT = FAILURE

    start = time.time()
    timestamp_vector = []
    first_touch = sys.float_info.max
    q.queue.clear()

    while True:

        # Exit if there are no touch events when timeout is reached
        if len(timestamp_vector) == 0 and time.time() - start > touch_timeout:
            print_to_console_and_logcat("Failure. Did not detect any touch events")
            OUTPUT = "Did not detect any touch events"
            return
        # Exit if we reach sleep timeout
        if time.time() - first_touch > max_sleep_duration:
            break
        # Exit if we reach a number of sufficient touch events
        elif len(timestamp_vector) > sufficient_events:
            break

        try:
            line = q.get_nowait()
        except Queue.Empty:
            continue

        match = json.loads(line)
        timestamp_vector.append(match["motion"]["timestamp"] / scale)
        if len(timestamp_vector) == 5:
            first_touch = time.time()
            print_to_console_and_logcat("Putting the device to sleep")

            for cmd in suspend_to_ram:
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
                out, err = p.communicate()
                if out:
                    print_to_console_and_logcat("Received error when trying to put device into sleep " + out)
                    exit_test()

    dev_state = subprocess.check_output(["adb get-state"], shell=True)
    if dev_state.strip("\n") != "device":
        print_to_console_and_logcat("Failure. Device did not recover; found device state: {}".format(dev_state))
        OUTPUT = "Failure. Adb state after sleep is: " + str(dev_state)
        exit_test()

    if len(timestamp_vector) < minimum_events:
        print_to_console_and_logcat("Failure. Not enough touch events were detected")
        OUTPUT = "Failure. Didn't receive enought touch events"
        return

    consecutive_elem_diff = [t - s for s, t in zip(timestamp_vector, timestamp_vector[1:])]
    time_diff_peak = max(consecutive_elem_diff)
    consecutive_elem_diff.remove(time_diff_peak)
    avg = numpy.mean(consecutive_elem_diff)
    std = numpy.std(consecutive_elem_diff)

    exp_max_std = 0.02
    exp_max_avg = 0.015
    avg_to_sleep_ratio = 200

    if (std < exp_max_std) and (avg < exp_max_avg) and (avg_to_sleep_ratio * avg < time_diff_peak):
        print_to_console_and_logcat("Successfully received poll events after sleep")
        VERDICT = SUCCESS
        OUTPUT = "Success"
    else:
        print_to_console_and_logcat("Did not detect sleep conditions. Average time diff between events: " +
                                    str(avg) + " Peak time difference: " + str(time_diff_peak) +
                                    " Standard deviation:" + str(std))
        OUTPUT = "Failure. Did not detect sleep conditions"

menu_items = ["Press enter when you are ready to begin the test",
              {"Move your finger continuously  in a circle on the touchscreen until further instructions": check_sleep},
              "You can stop moving your finger now. Press enter to continue",
              {"Do you want to repeat the test ? (y/n)": validate_result}
              ]

menu(menu_items, menu_items[1:])
