#!/usr/bin/env python

######################################################################
#
# @filename:    add_network_discovery.py
# @description: Tests that other networks are available when DUT is already connected.
#
# @run example:
#
#    python add_network_discovery.py -s 0BA8F2A0
#                                       --script-args
#                                           aps=Guest,EmployeeHotspot
#                                           mode=n
#                                           security=wpa_psk
#                                           encryption=aes
#                                           ap_name=ddwrt
#                                           passphrase=test1234
#                                           dut_security=wpa
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

#aps = args["aps"].split(",")
mode = args["mode"]
security = args["security"]
dut_security = args["dut_security"]
ddwrt_ap_name =  args["ap_name"]
status = args["status"]
link_speed = args["link_speed"]
frequency = args["frequency"]

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
encryption = None
if "encryption" in args.keys():
    encryption = args["encryption"]
ddwrt_ap_pass = None
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]

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

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial=serial)()

# aconnect to the Wi-Fi network
wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check network config
wifi_generic_steps.check_network_information(ap_name=ddwrt_ap_name,
                                    status="Connected",
                                    link_speed="Mbps",
                                    frequency="2.4",
                                    security="None",
                                    serial = serial)()
##### test end #####
