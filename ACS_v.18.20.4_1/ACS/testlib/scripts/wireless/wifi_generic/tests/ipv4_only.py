#!/usr/bin/env python

######################################################################
#
# @filename:    ipv4_only.py
# @description: Tests that DUT is able to connect an AP with IPv4 only protocol
#
# @run example:
#
#            ipv4_only.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       passphrase=test1234
#                                                       ipv6_enable=0
#                                                       radvd_enable=0
#                                                       ipv6_enable0=0
#                                                       ssid=dd-wrt-Mihai
#                                                       dut_security=WPA
#
#
# @author:      mihalachex.micu@intel.com
#
#######################################################################

##### imports #####
import sys
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
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
ddwrt_ap_name = args["ap_name"]
ddwrt_ap_pass = args['passphrase']
dut_security = args['dut_security']
encryption = args["encryption"]
ipv6_enable = args["ipv6_enable"]
radvd_enable = args["radvd_enable"]

#configure the AP

ap_steps.setup(mode = mode,
               new_ssid = ddwrt_ap_name,
               encryption = encryption,
               security = security,
               wifi_password = ddwrt_ap_pass,
               ipv6_enable = ipv6_enable,
               radvd_enable = radvd_enable)()


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


##### test end #####
