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
import re
import os
import json
import sys
from threading import Timer

# Adding location of util.py to the sys.path in order to import it in the testcase file
# Note that you must import 'util' module after importing this current module
util_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if util_path not in sys.path:
    sys.path.append(util_path)
from util import print_to_console_and_logcat

def retrieve_resolution_geteven():
    """
    Retrieves min and max coordinates (left top corner point and bottom right corner point) for resolution from getevent tool
    :return: max x, min x, max y, min y
    """
    cmd = 'adb shell getevent -li'
    proc = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE)
    stdout, stderr = proc.communicate()

    for line in stdout.split(os.linesep):
        if re.findall("ABS_MT_POSITION", line):
            if "ABS_MT_POSITION_X" in line:
                x_max = re.findall("max [0-9]*", line)
                x_max = int(x_max[0].split()[1])
                x_min = re.findall("min [0-9]*", line)
                x_min = int(x_min[0].split()[1])
            if "ABS_MT_POSITION_Y" in line:
                y_max = re.findall("max [0-9]*", line)
                y_max = int(y_max[0].split()[1])
                y_min = re.findall("min [0-9]*", line)
                y_min = int(y_min[0].split()[1])

    return x_max, x_min, y_max, y_min

def poll_touchscreen(events=1, timeout=10, stop_finger_up=False):
    """
    Polling on the touchtest tool for a number of events or until the timeout expires
    :param events: number of events to poll until exit
    :param timeout: maximum poll time
    :return: a list of events in json format
    """
    cmd = 'adb shell touchtest'
    proc = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE)
    timer = Timer(timeout, proc.kill)
    timer.start()
    events_structure = Touch_Events()
    index = 1
    for line in iter(proc.stdout.readline, ''):
        if index >= events:
            if timer.is_alive():
                timer.cancel()
            proc.kill()

        events_structure.extract_json(line)
        if stop_finger_up and events_structure.action[index-1] == "up":
            proc.kill()
        index += 1

    if index == 1:
        print_to_console_and_logcat("No data results from touchtest poll")
        return None
    return events_structure

def is_coord_in_corner(x, y, corner, pixel_margin=50):
    x_max, x_min, y_max, y_min = retrieve_resolution_geteven()
    if corner == "upper-left":
            corner_coord = (x_min, y_min)
    elif corner == "upper-right":
            corner_coord = (x_max, y_min)
    elif corner == "bottom-left":
            corner_coord = (x_min, y_max)
    elif corner == "bottom-right":
            corner_coord = (x_max, y_max)
    else:
        return False
    x_corner, y_corner = corner_coord
    if not (abs(x-x_corner) < pixel_margin and abs(y-y_corner) < pixel_margin):
        print_to_console_and_logcat("Coordinate [{}, {}] is not near the {} corner ([{}, {}])"
                                    .format(x, y, corner, x_corner, y_corner))
        return False
    return True

class Touch_Events():
    """
    Structure that contains the values for all events from json events
    In order to populate the structure, call extract_json on every line obtained from poll_touchscreen
    """

    def __init__(self):
        self.fingers_number = []
        self.timestamp = []
        self.action = []
        self.x = []
        self.y = []
        self.pressure = []
        self.size = []
        self.orientation = []

    def extract_json(self, json_line):
        """
        Extracts data from the json line and adds them to the Touch_Events structure
        :param json_line:input json line
        :return:
        """
        line = json.loads(json_line)
        self.fingers_number.append(len(line["motion"]["fingers"]))
        self.timestamp.append(line["motion"]["timestamp"])
        self.action.append(line["motion"]["action"])
        x_list = []
        y_list = []
        pressure_list = []
        size_list = []
        orientation_list = []
        for fingers in range(self.fingers_number[-1]):
            x_list.append(line["motion"]["fingers"][fingers]["x"])
            y_list.append(line["motion"]["fingers"][fingers]["y"])
            pressure_list.append(line["motion"]["fingers"][fingers]["pressure"])
            size_list.append(line["motion"]["fingers"][fingers]["size"])
            orientation_list.append(line["motion"]["fingers"][fingers]["orientation"])

        self.x.append(x_list)
        self.y.append(y_list)
        self.pressure.append(pressure_list)
        self.size.append(size_list)
        self.orientation.append(orientation_list)

    def get_all(self):
        return self.fingers_number, self.timestamp, self.action, self.x, self.y, self.pressure, self.size, self.orientation

    def debug_print(self):
        for fingers in self.fingers_number:
            print "Finger", self.fingers_number
            print "Timestamp:", self.timestamp
            print "Action:", self.action
            print "x:", self.x
            print "y:", self.y
            print "Pressure:", self.pressure
            print "Size:", self.pressure
            print "Orientation:", self.orientation
            print "-" * 100
