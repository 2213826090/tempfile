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

import sys
import os
import subprocess
import json
import numpy
from threading import Timer
from sensors_util import get_sensor_hal_info
from sensors_util import sens2_out
from util import print_test_result
from util import exit_test

arg = sys.argv

poll_time = 35
timeout = poll_time * 3 + 5

try:
    sensor_info = get_sensor_hal_info(arg[1])
    sensor_id = sensor_info["index"]
    sensor_type = sensor_info["type"]

except Exception:
    print_test_result(False, "Sensor was not found")
    exit_test()

delay = sensor_info["minDelay"]
sensor_frequency = 1000000.00 / delay

cmd = ['adb', 'shell', 'sens2', 'poll', str(sensor_id)]
aux_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
proc = subprocess.Popen(['tee', sens2_out], stdin=aux_proc.stdout, stdout =subprocess.PIPE)
timeout = 20
timer = Timer(timeout, aux_proc.kill)
timer.start()
timestamp_vector = []

for line in iter(proc.stdout.readline, ''):
    match = json.loads(line)
    timestamp_vector.append(match["timestamp"])
    if len(timestamp_vector) == 5:
        p1 = subprocess.Popen(["adb", "shell", "echo", "devices", ">", "/sys/power/pm_test"], stdout=subprocess.PIPE)
        out1, err1 = p1.communicate()
        if out1:
            print_test_result(False, "Received error when trying to put device into sleep " + out1)
            exit_test()
        p2 = subprocess.Popen(["adb", "shell", "echo", "reboot", ">", "/sys/power/disk"], stdout=subprocess.PIPE)
        out2, err2 = p2.communicate()
        if out2:
            print_test_result(False, "Received error when trying to put device into sleep " + out2)
            exit_test()
        p3 = subprocess.Popen(["adb", "shell", "echo", "disk", ">", "/sys/power/state"], stdout=subprocess.PIPE)
        out3, err3 = p3.communicate()
        if out3:
            print_test_result(False, "Received error when trying to put device into sleep " + out3)
            exit_test()
    # Measure 10 seconds.
    elif len(timestamp_vector) > int(sensor_frequency) * 10:
        if timer.is_alive():
            timer.cancel()
        aux_proc.kill()
        break

consecutive_elem_diff = [t - s for s, t in zip(timestamp_vector, timestamp_vector[1:])]
time_diff_peak = max(consecutive_elem_diff)
consecutive_elem_diff.remove(time_diff_peak)
avg = sum(consecutive_elem_diff) / len(consecutive_elem_diff)
std = numpy.std(consecutive_elem_diff)

# less than 0.5
if (std < 500000000) and (2 * avg < time_diff_peak):
    print_test_result(True, "Successfully received poll events after sleep")
else:
    print_test_result(False,
                      "Did not detect sleep conditions. Average time diff between events: " + str(
                          avg) + " Peak time difference: " + str(time_diff_peak) + " Standard deviation:" + str(std))
