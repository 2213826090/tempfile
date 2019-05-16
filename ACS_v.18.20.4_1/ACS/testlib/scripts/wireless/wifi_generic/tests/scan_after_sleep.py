#!/usr/bin/env python

######################################################################
#
# @filename:    scan_after_sleep.py
# @description: Tests that scannig is done after sleep.
#
# @run example:
#
#    python scan_after_sleep.py -s 0BA8F2A0
#                                       --script-args
#                                           ap_name='Android Core QA'
#
# @author:      corneliu.stoicescu@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

mode = args["mode"]
security = args["security"]

new_ssid = None
if "new_ssid" in args.keys():
    new_ssid = args["new_ssid"]
# mandatory params
ap_name = args["ap_name"]

#ap_steps.setup(mode, security, new_ssid = new_ssid if str(new_ssid).lower() == "none" else new_ssid, serial = serial)()
ap_steps.setup(mode, security,
               new_ssid = ap_name,
               serial = serial)()

##### test start #####
adb_steps.connect_device(serial=serial,
                         port=adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# check we are disconnected from AP
wifi_generic_steps.check_wifi_state_disconnected(ap_name=ap_name, serial=serial)()

# Check we can scan and find the AP
wifi_generic_steps.scan_and_check_ap(serial=serial, ap=ap_name, should_exist=True)()

# Put the device to sleep
ui_steps.put_device_into_sleep_mode(serial=serial)()

# Set the wireless interface on the AP down
ap_steps.set_ap_wireless(state="down")()

# Wait for 10 seconds
time.sleep(10)

# Wake up the device
ui_steps.wake_up_device(serial=serial)()

# Unlock the device using swipe
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# Click home
ui_steps.press_home(serial = serial)()
# Scan for the AP and check it's not there.
wifi_generic_steps.scan_and_check_ap(serial=serial, ap=ap_name, should_exist=False)()

# Set the AP wireless interface back up.
ap_steps.set_ap_wireless(state="up")()
