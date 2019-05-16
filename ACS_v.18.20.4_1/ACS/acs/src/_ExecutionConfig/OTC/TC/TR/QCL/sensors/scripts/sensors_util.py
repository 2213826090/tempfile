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
import json
import sys
import os
import difflib
import math
import numpy
from threading import Thread
from threading import Timer

# Adding location of util.py to the sys.path in order to import it in the testcase file
# Note that you must import 'util' module after importing this current module
util_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if util_path not in sys.path:
    sys.path.append(util_path)
from util import print_to_console_and_logcat
from util import exit_test

sens2_out = '/tmp/sens2_out'

########################################################################
# Define thresholds based on expected frequency
# All measurements should (most important is listed first):
#       - not exceed 30s
#       - be at least 10 seconds long for reasonable average value AND
#       - have at least 50 samples
########################################################################
def define_thresholds(frequency, max_duration=30, minimum_samples=150.0, minimum_duration=10):
    estimated_duration = minimum_samples / frequency
    return numpy.min([max_duration, numpy.max([minimum_duration, estimated_duration])])

def is_sd_as_expected(values_array):
    expected_standard_deviation = 0.5
    standard_deviation = numpy.std(values_array)
    result = {}
    result["message"] = "maximum expected sd " + str(expected_standard_deviation) + \
                        ", obtained sd " + str(standard_deviation)
    if standard_deviation < expected_standard_deviation:
        result["test_passed"] = True
    else:
        result["test_passed"] = False
    return result

def get_sensor_hal_info(info_string):
    info_string = info_string.replace("\"", "")
    info_string = info_string.replace("\'", "\"")
    component_json = json.loads(info_string)
    if component_json["type"] == "sensor":
        sensor_type = component_json["sensor"]["type"]
        sensor_name = component_json["id"]["name"].lower()
        try:
            proc = subprocess.Popen(['adb', 'shell', 'sens2', 'ls'], stdout=subprocess.PIPE)
            max_ratio = -1
            other_type_max_ratio = 0
            matched_item = None
            other_type_matched_item = None
            for line in iter(proc.stdout.readline, ''):
                parsed_json = json.loads(line.rstrip())
                parsed_json_name = parsed_json["name"].lower()
                match = difflib.SequenceMatcher(None, sensor_name, parsed_json_name)
                match_ratio = match.ratio()
                if (sensor_type == parsed_json["type"]):
                    if (parsed_json_name.find(sensor_name) != -1):
                        return parsed_json
                    elif (parsed_json_name.find(sensor_name.replace(" ", "")) != -1):
                        return parsed_json
                    elif match_ratio > max_ratio:
                        max_ratio = match_ratio
                        matched_item = parsed_json
                elif match_ratio > other_type_max_ratio:
                    other_type_max_ratio = match_ratio
                    other_type_matched_item = parsed_json
            if matched_item != None:
                return matched_item
            elif other_type_max_ratio > 0.4:
                return other_type_matched_item
        except:
            if proc:
                proc.kill()
            return None
    return None

def poll_for_events(sensor_id, sensor_frequency, timeout, seconds=0, nr_events=0):
    cmd = ['adb', 'shell', 'sens2', 'poll']
    if sensor_id != None:
        id_freq = str(sensor_id)
        #hardcoded for emulator
        if sensor_frequency == 0 or sensor_frequency > 30:
            sensor_frequency = 30
        if sensor_frequency > 0:
            id_freq += "," + str(sensor_frequency)
        cmd.append(id_freq)
    try:
        timestamp_int = math.pow(10, 9) * seconds
        aux_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        proc = subprocess.Popen(['tee', sens2_out], stdin=aux_proc.stdout, stdout=subprocess.PIPE)
        timer = Timer(timeout, aux_proc.kill)
        timer.start()
        events = []
        index = 0
        for line in iter(proc.stdout.readline, ''):
            parsed_json = json.loads(line.rstrip())
            events.append(parsed_json)
            index = index + 1
            if (nr_events != 0 and index >= nr_events or seconds != 0 and index > 0 and \
                            events[index - 1]["timestamp"] - events[0]["timestamp"] > timestamp_int):
                if timer.is_alive():
                    timer.cancel()
                aux_proc.kill()
                return events
        if (seconds != 0 and nr_events == 0 and index == 1):
            return events
        return []
    except:
        if aux_proc:
            aux_proc.kill()
        exit_test()

def get_period_values(events_list, sensor_type):
    p = math.pow(10, 9)
    timestamp = []
    start_index = 0
    end_index = 0
    period_values = []
    for event in events_list:
        if event['type'] == sensor_type:
            timestamp.append(event['timestamp'])
    seconds = [x / float(p) for x in timestamp]
    periods = [x - seconds[i - 1] for i, x in enumerate(seconds)][1:]
    return periods

def get_axis_values(axis, events_list, sensor_type):
    values = []
    for event in events_list:
        if event['type'] == sensor_type:
            values.append(event["data"][axis])
    return values

# Classes
class PollThread(Thread):
    '''
        A thread that reads data at the maximum frequency from the self.axes and
        writes them on a single line.
        Need to overwrite self.axes, self.id, and self.type to work.
    '''

    def __init__(self):
        self.axes = None
        self.id = None
        self.type = None
        self.proc = None
        self.aux_proc = None
        self.values = []
        Thread.__init__(self)

    def run(self):
        try:
            self.values = []
            self.aux_proc = subprocess.Popen(['adb', 'shell', 'sens2', 'poll', str(self.id)], stdout=subprocess.PIPE)
            self.proc = subprocess.Popen(['tee', sens2_out], stdin=self.aux_proc.stdout, stdout =subprocess.PIPE)

            for line in iter(self.proc.stdout.readline, ''):
                parsed_json = json.loads(line.rstrip())
                if parsed_json['type'] == self.type:
                    sys.stdout.write(str(parsed_json['data'][self.axes]) + " \r")
                    self.values.append(float(parsed_json['data'][self.axes]))

        except Exception as e:
            print_to_console_and_logcat("Exception in PollThread's run")
            exit_test()

    def stop(self):
        '''
           Called when stopping the thread
        '''
        return

    def check_values(self):
        '''
            Called when validating the values from the poll
        '''
        return
