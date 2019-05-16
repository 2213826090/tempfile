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

from threading import Timer
import os
import json
import subprocess
import display_util
from util import print_to_console_and_logcat
from util import print_test_result
from util import is_frequency_in_expected_range
from util import exit_test

expected_frequency = 60

# poll for 10 seconds
expected_nr_events = expected_frequency * 10

def poll_for_events(rate=1, nr_events=expected_nr_events, timeout=20):
    cmd = ['adb', 'shell', 'display_test', 'vsync_test', str(rate)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    timer = Timer(timeout, proc.kill)
    timer.start()
    events = []
    index = 0
    for line in iter(proc.stdout.readline, ''):
        parsed_json = json.loads(line.rstrip())
        events.append(parsed_json)
        index = index + 1
        if index >= nr_events:
            if timer.is_alive():
                timer.cancel()
            proc.kill()
            return events
    return events

try:
    events = poll_for_events()
    if len(events) < expected_nr_events:
        print_test_result(False, "Timeout excedeed")
    else:
        period_values = []
        # get periods array in seconds
        for event in events[1:]:
            period_values.append(event["event vsync"]["period"] / 1000)
        result = is_frequency_in_expected_range(expected_frequency, period_values)
        print_test_result(result["test_passed"], result["message"])
except Exception as e:
    print_to_console_and_logcat(str(e))
    print_test_result(False, "Vsync poll could not be realised")
    exit_test()
