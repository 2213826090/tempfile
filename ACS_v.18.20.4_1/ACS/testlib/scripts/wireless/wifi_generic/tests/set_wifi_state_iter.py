#!/usr/bin/env python

######################################################################
#
# @filename:    set_wifi_state_iter.py
# @description: Tests that when switch wifi state On and Off.
#
# @run example:
#
#            python set_wifi_state_iter.py -s 0BA8F2A0
#                                          --script-args
#                                               iterations = n
#
# @author:      corneliu.stoicescu@intel.com
#
#######################################################################
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.wireless.wifi import wifi_steps
import sys
import time

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
iterations = int(args.get("iterations"))
if "use_adb" in args.keys():
   use_adb=args["use_adb"]
else:
   use_adb="True"
if "open_settings" in args.keys():
  print "in if loop"
  open_s=args["open_settings"]
else:
  open_s="True"
# turn display on, if turned off
ui_steps.wake_up_device(serial=serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial=serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial=serial)()
wifi_steps.open_wifi_settings(serial=serial)()
# verify the WiFi can be enabled/disabled iterative
x=time.time()
for iteration in range(iterations):
    print "iteration -"
    print iteration
    print open_s
    wifi_steps.set_wifi_state(state="OFF", use_adb=use_adb,open_settings=open_s,serial=serial)()
    wifi_steps.set_wifi_state(state="ON", use_adb=use_adb,open_settings=open_s,serial=serial)()
#    wifi_steps.set_from_wifi_settings(state="OFF", intent=use_adb,open_settings=open_s,serial=serial)()
#    wifi_steps.set_from_wifi_settings(state="ON", intent=use_adb,open_settings=open_s,serial=serial)()
y=time.time()
time=int(y)-int(x)
print time
#print "time taken to complete"+iterations+"is"+time
