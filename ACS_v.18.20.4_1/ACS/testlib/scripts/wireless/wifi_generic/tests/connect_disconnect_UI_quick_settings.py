#!/usr/bin/env python

######################################################################
#
# @filename:    connect_disconnect_UI_quick_settings.py
# @description: Tests that the DUT can connect to a network and disconnect
#               from it (iteratively) using the WiFi quick settings UI.
#
# @run example:
#
#            python connect_disconnect_UI_quick_settings.py -s 0BA8F2A0
#                            --script-args
#                                mode=n
#                                ap_name=ddwrt
#                                passphrase=test1234
#
# @author:      stefanx.todirica@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_steps
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

# mandatory params
mode = args["mode"]
ddwrt_ap_name =  args["ap_name"]
ddwrt_ap_pass = args["passphrase"]
security = "wpa2"
encryption = "aes"

iterations = 1
if "iterations" in args.keys():
    iterations = args["iterations"]


##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# configure ap
ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               new_ssid = ddwrt_ap_name,
               serial = serial)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# open wifi settings menu
wifi_generic_steps.set_wifi(serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# start the stress test
wifi_steps.connect_disconnect_UI_stress_2(ap_name = ddwrt_ap_name,
                               password = ddwrt_ap_pass,
                               iterations = iterations,
                               serial = serial)()

# go to home screen
ui_steps.press_home(serial = serial)()

##### test end #####
