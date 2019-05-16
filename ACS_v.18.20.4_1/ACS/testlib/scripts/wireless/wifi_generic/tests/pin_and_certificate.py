#!/usr/bin/env python

######################################################################
#
# @filename:    pin_and_certificate.py
# @description: Pre-requisite step for WiFi tests with EAP-TLS (use certificate).
#
# @run example:
#
#            python pin_and_certificate.py -s 0BA8F2A0
#                                               --script-args
#                                                       dut_pin=1234
#                                                       cert_pass=whatever
#                                                       cert_name=TLS_certificate
#
# @author:      stefanx.todirica@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
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
dut_pin = None
if "dut_pin" in args.keys():
    dut_pin = args["dut_pin"]
cert_pass = None
if "cert_pass" in args.keys():
    cert_pass = args["cert_pass"]
cert_name = None
if "cert_name" in args.keys():
    cert_name = args["cert_name"]
wait_time = 5
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

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# set screen lock to PIN
ui_steps.set_pin_screen_lock(serial = serial, dut_pin = dut_pin, wait_time = int(wait_time)*1000)()

# install certificate
wifi_generic_steps.install_WIFI_certificate(serial = serial, cert_pass=cert_pass, cert_name=cert_name, dut_pin = dut_pin, wait_time = wait_time)()
