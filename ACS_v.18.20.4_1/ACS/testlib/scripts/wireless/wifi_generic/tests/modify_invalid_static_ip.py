#!/usr/bin/env python

######################################################################
#
# @filename:    modify_invalid_static_ip.py
# @description: This test will verify that an invalid static address won't be accepted.
#
#
# @run example:
#
#            python modify_invalid_static_ip.py -s 0BA8F2A0
#                                               --script-args
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#                                                       dut_security=wpa
#                                                       false_ip_addresses=192.168.1.256,!@#$%^*(),qwerty
#
# @author:      vlad.a.gruia@intel.com
#
#######################################################################


##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
dut_security = args["dut_security"]
ddwrt_ap_name =  args["ap_name"]
ddwrt_ap_pass = None
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]
false_ip_addresses = args["false_ip_addresses"].split(",")

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# verify that the IP addresses are invalid
for false_ip_address in false_ip_addresses:
    wifi_generic_steps.modify_network(ssid = ddwrt_ap_name,
                                      security = dut_security,
                                      password = ddwrt_ap_pass,
                                      serial = serial,
                                      ip_settings = "Static",
                                      ip_address = false_ip_address,
                                      valid_config = False)()

##### test end #####
