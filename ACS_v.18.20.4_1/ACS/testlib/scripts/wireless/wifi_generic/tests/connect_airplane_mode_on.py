#!/usr/bin/env python

######################################################################
#
# @filename:    connect_airplane_mode_on.py
# @description: Tests that wifi can be enabled when airplane mode is on.
#               connect to it.
#
# @run example:
#
#            python connect_airplane_mode_on.py -s 0BA8F2A0
#                                               --script-args
#                                                       ap_name=ddwrt
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
ap_name =  args["ap_name"]

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()


# airplane mode on
wifi_generic_steps.set_airplane_mode(state = "ON", serial = serial)()

# wifi ON
wifi_generic_steps.set_wifi(state="ON", serial = serial)()

# scan and look for ap_name
wifi_generic_steps.scan_and_check_ap(serial = serial,
                                     ap=ap_name)()

##### test end #####
