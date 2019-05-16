#!/usr/bin/env python

######################################################################
#
# @filename:    unlock.py
# @description: Unlock device and remove pin lock & certificates, if configured
#
# @run example:
#              python unlock.py -s 4CAA34A7
#                    --script-args
#                    pin=1234
#                    serial2=599433A7
#
# @author:      alexandru.i.nemes@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults


##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
serial2 = None
if "serial2" in args.keys():
    serial2 = args["serial2"]

serial_list = [serial, serial2]

##### test start #####

for serial in serial_list:
    if serial:
        device_connected = False
        try:
            adb_steps.connect_device(serial = serial)()
            device_connected = True
            # only execute steps if device with "serial" is actually connected
        except:
            device_connected = False
        if device_connected:
            # unlock devices
            adb_steps.command(serial = serial, timeout = 10,
                            command = "input keyevent 82")()
            ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

            sys.exit()
            wifi_steps.remove_certificates(serial = serial, pin=wifi_defaults.wifi['pin'])()

            ui_steps.remove_pin_screen_lock(serial = serial,
                                    dut_pin = wifi_defaults.wifi['pin'], new_mode = "None")()

            # go to home screen
            ui_steps.press_home(serial = serial)()
