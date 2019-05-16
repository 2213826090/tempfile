#!/usr/bin/env python

######################################################################
#
# @filename:    verify_scan_works.py
# @description: Tests that scannig is done.
#
# @run example:
#
#    python verify_scan_works.py -s 0BA8F2A0
#                                       --script-args
#                                           aps=Guest,EmployeeHotspot,'Android Core QA'
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

dummy_SSID = "dummy"
ddwrt_ap_name = args["ap_name"]
aps = [ddwrt_ap_name]

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# add dummy Wi-Fi network
wifi_generic_steps.add_network(ssid = dummy_SSID,
                               security = "WPA",
                               password = "test1234",
                               serial = serial)()

# check the networks are scanned
for ap_name in aps:
    wifi_generic_steps.scan_and_check_ap(serial = serial,
                                     ap=ap_name)()

# remove all networks
wifi_generic_steps.clear_saved_networks(serial = serial)()


# check the networks are scanned
for ap_name in aps:
    wifi_generic_steps.scan_and_check_ap(serial = serial,
                                     ap=ap_name)()

##### test end #####
