#!/usr/bin/env python

######################################################################
#
# @filename:    post-reqs.py
# @description: Ensures that the DUT goes back to its default state
#
# @run example:
#
#            python post-reqs.py -s 0BA8F2A0
#                                   --script-args
#                                           mode=certificate
#                                           dut_pin=1324
#
# @author:      stefanx.todirica@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
postreq = args["postreq"]

# optional params
dut_pin = None
if "dut_pin" in args.keys():
    dut_pin = args["dut_pin"]
mode = None
if "mode" in args.keys():
    mode = args["mode"]
security = None
if "security" in args.keys():
    security = args["security"]
encryption = None
if "encryption" in args.keys():
    encryption = args["encryption"]
ddwrt_ap_pass = None
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]
ddwrt_ap_name = None
if "ap_name" in args.keys():
    ddwrt_ap_name = args["ap_name"]
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

# POST-REQ certificate
if postreq == "certificate":

    # remove certificate
    wifi_generic_steps.remove_certificates(serial = serial, wait_time = wait_time, pin = dut_pin)()

    # remove screen lock PIN security
    ui_steps.remove_pin_screen_lock(serial = serial, dut_pin = dut_pin, wait_time = int(wait_time)*1000)()

# POST-REQ AP ssid & channel bandwidth
elif postreq == "reset_ap":
    ap_steps.set_ap_wireless(state = "up")()
    ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               new_ssid = ddwrt_ap_name,
               serial = serial)()
    ap_steps.setup_virtual_interface(False,
               interface5ghz = '0',
               serial=serial)()

# POST-REQ AP's 5GHz ssid & channel bandwidth
elif postreq == "reset_ap_5":
    ap_steps.set_ap_wireless(state = "up")()
    ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               new_ssid = ddwrt_ap_name+'5',
               interface5ghz = '1',
               serial = serial)()
    ap_steps.setup_virtual_interface(False,
               interface5ghz = '1',
               serial=serial)()

# POST-REQ for frequency band (DUT) -> sets it to Automatic
elif postreq == "dut_frecuency_band":
    wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = 'auto', verify_dumpsys = False)()
