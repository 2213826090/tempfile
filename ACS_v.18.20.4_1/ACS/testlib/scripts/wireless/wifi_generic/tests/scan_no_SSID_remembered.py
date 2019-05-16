#!/usr/bin/env python

######################################################################
#
# @filename:    scan_no_SSID_remembered.py
# @description: Tests that scannig is done.
#
# @run example:
#
#            python add_network_and_connect.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#                                                       dut_security=wpa
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
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.utils.defaults import wifi_defaults
from testlib.utils.statics.android import statics

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
iterations = 1
if "iterations" in args.keys():
    iterations = args["iterations"]

device_info = statics.Device(serial = serial)

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

# turn wifi on and off
wifi_generic_steps.set_wifi(state="OFF", serial = serial)()
wifi_generic_steps.set_wifi(state="ON", serial = serial)()

# check the DUT is scanning
count = 0
while count < int(iterations):
    time.sleep(5)
    if device_info.dessert in ["N", "O"]:
        wifi_generic_steps.check_connection_info(serial=serial,
                                                 state='DISCONNECTED/DISCONNECTED',
                                                 timeout=10)()
    else:
        wifi_generic_steps.check_connection_info(serial = serial,
                                                state='DISCONNECTED/SCANNING',
                                                timeout=10)()
    count += 1


##### test end #####
