#!/usr/bin/env python

######################################################################
#
# @filename:    reconnect_after_ap_reconfig.py
# @description: Tests that the device can connect/reconnect to the AP
#               after AP is reconfigured with/without device reconfiguration.
#
# @run example:
#
#            python reconnect_after_ap_reconfig.py -s 0BA8F2A0
#                                                  --script-args
#                                                  mode1=n
#                                                  mode2=n
#                                                  security1=wpa_psk
#                                                  security2=wpa_psk
#                                                  encryption1=aes
#                                                  encryption2=aes
#                                                  ap_name=ddwrt
#                                                  passphrase1=test1234
#                                                  passphrase2=test1234
#                                                  dut_security1=wpa
#                                                  dut_security2=wpa
#                                                  reconfigure_network=True
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

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
mode1 = args["mode1"]
mode2 = args["mode2"]
security1 = args["security1"]
security2 = args["security2"]
dut_security1 = args["dut_security1"]
dut_security2 = args["dut_security2"]
ddwrt_ap_name =  args["ap_name"]
reconfigure_network = (args["reconfigure_network"] == "True")
conf_security1 = args["conf_security1"]
conf_security2 = args["conf_security2"]
pairwise_cipher1 = args["pairwise_cipher1"]
pairwise_cipher2 = args["pairwise_cipher2"]

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
encryption1 = None
if "encryption1" in args.keys():
    encryption1 = args["encryption1"]
ddwrt_ap_pass1 = None
if "passphrase1" in args.keys():
    ddwrt_ap_pass1 = args["passphrase1"]
encryption2 = None
if "encryption2" in args.keys():
    encryption2 = args["encryption2"]
ddwrt_ap_pass2 = None
if "passphrase2" in args.keys():
    ddwrt_ap_pass2 = args["passphrase2"]



##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# configure ap with the first settings set
ap_steps.setup(mode1, security1,
               encryption = encryption1,
               wifi_password = ddwrt_ap_pass1,
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

# add the Wi-Fi network
wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security1,
                               password = ddwrt_ap_pass1,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED',
                                        Security=conf_security1,
                                        pairwise_cipher=pairwise_cipher1)()


# configure ap with the second settings set
ap_steps.setup(mode2, security2,
               encryption = encryption2,
               wifi_password = ddwrt_ap_pass2,
               serial = serial)()

if reconfigure_network:
    # Add the wi-fi network again, with the new settings
    wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                                security = dut_security2,
                                password = ddwrt_ap_pass2,
                                serial = serial)()

wifi_generic_steps.set_wifi(state="OFF", serial = serial)()
wifi_generic_steps.set_wifi(state="ON",serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network
wifi_generic_steps.check_connection_info(serial = serial,
                                         SSID = ddwrt_ap_name,
                                         state='CONNECTED/CONNECTED',
                                         Security=conf_security2,
                                         pairwise_cipher=pairwise_cipher2,
                                         timeout=60)()


##### test end #####
