#!/usr/bin/env python

######################################################################
#
# @filename:    network_renew_ip_address.py
# @description: Tests that the DUT is renew the IP base on dhcp_lease that is set on the AP
#
# @run example:
#
#            network_renew_ip_address.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       passphrase=test1234
#                                                       dhcp_lease=2
#                                                       ssid=dd-wrt-Mihai
#
# @author:      mihalachex.micu@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.android.logcat import logcat_steps
##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val



# mandatory params
mode = args["mode"]
security = args["security"]
ddwrt_ap_name = args["ap_name"]
ddwrt_ap_pass = args['passphrase']
dut_security = args['dut_security']
dhcp_lease_time = args['dhcp_lease_time']

# optional params

ddwrt_ap_pass = None
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]

encryption = None
if "encryption" in args.keys():
    encryption = args["encryption"]

#configure the AP

ap_steps.setup(dhcp_lease = dhcp_lease_time,
               new_ssid = ddwrt_ap_name,
               mode = mode,
               encryption = encryption,
               security = security,
               wifi_password = ddwrt_ap_pass)()


##### test start #####

#clear the logcat

logcat_steps.clear_logcat(serial = serial)()

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

# add a network
wifi_generic_steps.add_network(ssid = ddwrt_ap_name, security = dut_security,
                               password = ddwrt_ap_pass,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network.
wifi_generic_steps.check_connection_info(serial = serial,
                                         SSID = ddwrt_ap_name,
                                         state='CONNECTED/CONNECTED')()

#verify in logcat for the message
wifi_generic_steps.check_lease_time(serial = serial, dhcp_lease_time=dhcp_lease_time)()

#ping the gateway
wifi_generic_steps.ping_gateway(serial = serial)()

##### test end #####
