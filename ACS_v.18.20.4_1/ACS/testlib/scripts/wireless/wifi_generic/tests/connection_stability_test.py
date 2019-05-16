#!/usr/bin/env python

######################################################################
#
# @filename:    test_connection_stability.py
# @description: Configure and connect to a network. After that check
#               connection stability by pinging and checking connection state.
#
# @run example:
#
#            python test_connection_stability.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#                                                       dut_security=wpa
#                                                       duration=60
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
mode = args["mode"]
security = args["security"]
dut_security = args["dut_security"]
ddwrt_ap_name =  args["ap_name"]
duration = args["duration"]

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
wifi_generic_steps.clear_saved_networks(serial = serial)()

# add the Wi-Fi network
wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

finish = time.time() + int(duration)
print "Runnign stability test for " + duration + " seconds"
while finish > time.time():

    # check we are connected to the correct network.
    wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED')()

    wifi_generic_steps.ping_gateway(serial = serial, trycount="3")()

    print "Connection status OK at " + str(finish - time.time())

##### test end #####
