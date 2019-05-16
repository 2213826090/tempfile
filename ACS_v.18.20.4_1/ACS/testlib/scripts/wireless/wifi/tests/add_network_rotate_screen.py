#!/usr/bin/env python

######################################################################
#
# @filename:    add_network_rotate_screen.py
# @description: Tests that when correctly adding a network the device will
#               connect to it.
#
# @run example:
#
#            python add_network_rotate_screen.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
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
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.adb import adb_utils

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
mode = args["mode"]
security = args["security"]
ddwrt_ap_name =  args["ap_name"]

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

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial)()

if adb_utils.get_product_name(serial=serial) == "r2_s3gr10m6s_trusty":
    wifi_steps.open_wifi_settings(serial=serial)()

# rotate screen
# get the device rotation type
ui_steps.set_orientation(serial = serial,
                     orientation = "landscape",
                     target = "tablet")()
device_type = adb_utils.get_device_orientation_type(serial = serial)

# rotate to portrait
ui_steps.set_orientation(serial = serial,
                     orientation = "portrait",
                     target = device_type)()


# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# connect to AP using the GUI and rotation on the screen during the process
wifi_steps.connect_with_pass_change_orientation(ap_name=ddwrt_ap_name,
                                                password=ddwrt_ap_pass,
                                                serial = serial,
                                                device_type=device_type)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network.
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED')()

##### test end #####
