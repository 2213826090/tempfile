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
import json
import os
import sys

# Adding location of util.py to the sys.path in order to import it in the testcase file
# Note that you must import 'util' module after importing this current module
util_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if util_path not in sys.path:
    sys.path.append(util_path)
from util import print_to_console_and_logcat

INCH_TO_MM_RATIO = 25.4

def get_native_timings(desc):
    if "display-timings" not in desc.keys():
        return None

    timings_desc = desc["display-timings"]
    if "native-mode" not in timings_desc:
        return None

    phandle = timings_desc["native-mode"]
    for key, val in timings_desc.items():
        if "linux,phandle" not in val:
            continue
        if phandle == val["linux,phandle"]:
            return val

def get_resolution(desc):
    if "intel,display-resolution" in desc.keys():
        return tuple(str(int(desc["intel,display-resolution"][1], 16)),
                     str(int(desc["intel,display-resolution"][2], 16)))

    native_timings = get_native_timings(desc)

    if native_timings is None:
        return (None, None)

    sizex = native_timings.get("vactive", [None] * 2)[1]
    sizey = native_timings.get("hactive", [None] * 2)[1]
    return (str(int(sizex, 16)), str(int(sizey, 16)))

def mm_to_inch(value):
    return float(value) / INCH_TO_MM_RATIO

def get_dpi(desc):
    if "intel,display-dpi" in desc.keys():
        return tuple(str(int(desc["intel,display-dpi"][1], 16)),
                     str(int(desc["intel,display-dpi"][2], 16)))
    sizes = get_resolution(desc)
    if None in sizes:
        return (None, None)

    native_timings = get_native_timings(desc)

    if native_timings is None:
        return (None, None)

    if "width" not in native_timings:
        dpix = None
    else:
        dpix = str(float(sizes[0]) /
                   mm_to_inch(int(native_timings["width"][1], 16)))

    if "height" not in native_timings:
        dpiy = None
    else:
        dpiy = str(float(sizes[1]) /
                   mm_to_inch(int(native_timings["height"][1], 16)))

    return (dpix, dpiy)

def get_desc_info(info_string, info_name):
    info_string = info_string.replace("\"", "")
    info_string = info_string.replace("\'", "\"")
    component_json = json.loads(info_string)
    try:
        if component_json["type"] == "display":
            desc = component_json["display"]["dt"].values()[0]
            if info_name == "fps":
                if "intel,display-fps" not in desc.keys():
                    return None
                info = str(int(desc["intel,display-fps"][1], 16))
            elif info_name == "xdpi":
                info = get_dpi(desc)[0]
            elif info_name == "ydpi":
                info = get_dpi(desc)[1]
            elif info_name == "width":
                info = get_resolution(desc)[0]
            elif info_name == "height":
                info = get_resolution(desc)[1]
            return info
        return None
    except:
        return None

def display_on_screen(color, timeout = 4):
    cmd = ['adb', 'shell', 'display_test', 'color_test']
    if color == 1:
        cmd.append('red')
    elif color == 2:
        cmd.append('green')
    elif color == 3:
        cmd.append('blue')
    else:
        # show white screen
        cmd.extend(['255', '255', '255', '0'])
    cmd.append(str(timeout))
    return subprocess.Popen(cmd, stdout=subprocess.PIPE)

def get_display_info():
    cmd = 'adb shell display_test info_test'
    proc = subprocess.Popen(shlex.split(cmd), stdout=subprocess.PIPE)
    line = proc.stdout.readline()
    return json.loads(line.rstrip())

def set_brightness(value):
    cmd = ['adb', 'shell', 'display_test', 'set_brightness', str(value)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)

def user_validation():
    while (True):
        input_line = raw_input("")
        if input_line == "y":
            return True
        elif input_line == "n":
            return False

def suspend_and_resume():
    cmd = ['adb', 'shell', 'display_test', 'sleep_test', 'suspend']
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    print "Please confirm if the display has been suspended. (y/n)"
    if not user_validation():
        return False
    cmd[-1] = 'resume'
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    return True

def validate_response(expected_color):
    colors = ["red", "green", "blue", "white"]
    while (True):
        try:
            input_line = int(raw_input(""))
        except:
            print "Your option should be an integer"
            continue
        if input_line == expected_color:
            return True, "Success"
        elif input_line in range(1, 5):
            message = "Failure. Expected color was " + colors[expected_color - 1] + \
                                        " and obtained color was " + colors[input_line - 1]
            return False, message
        elif input_line == 5:
            message = "Failure. Expected color was " + colors[expected_color - 1] + \
                                        " and obtained color was other than the options"
            return False, message
        elif input_line == 0:
            display_on_screen(expected_color)
        else:
            print str(input_line) + " is not a valid option"
