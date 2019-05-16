#!/usr/bin/env python

######################################################################
#
# @filename:    toggle_airplane_switch_settings.py
# @description: Tests if the airplane mode switch from the Settings menu
#               works correctly.
#
# @run example:
#
#            python toggle_airplane_switch_settings.py -s 0BA8F2A0
#                                               --script-args
#                                                       iterations=10
#                                                       wait_time=5
#
# @author:      stefanx.todirica@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args:
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val

# optional params
iterations = 1
if "iterations" in args.keys():
    iterations = args["iterations"]
wait_time = 10
if "wait_time" in args.keys():
    wait_time = args["wait_time"]

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# start toggling the switch
wifi_steps.toggle_airplane_mode_settings(serial = serial, wait_time = wait_time, iterations = iterations)()

# go to home screen
ui_steps.press_home(serial = serial)()

##### test end #####
