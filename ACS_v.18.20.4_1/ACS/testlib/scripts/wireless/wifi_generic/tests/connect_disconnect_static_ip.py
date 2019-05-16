#!/usr/bin/env python

######################################################################
#
# @filename:    connect_disconnect_static_ip.py
# @description: Tests that network can be connected and disconnected, uses static IP
#
# @run example:
#
#            python connect_disconnect_static_ip.py -s 0BA8F2A0
#                            --script-args
#                                mode=n
#                                security=wpa_psk
#                                encryption=aes
#                                ap_name=ddwrt
#                                passphrase=test1234
#                                dut_security=wpa
#
# @author:      vlad.a.gruia@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_utils
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
if "static_ip_range" in args.keys():
    static_ip_range = args["static_ip_range"]
else:
    static_ip_range = None

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

# connect to the AP via DHCP
wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED')()
wifi_generic_steps.ping_gateway(serial = serial)()

# get connection info from current DHCP connection
content = wifi_utils.get_connection_content(serial = serial)
connection_info = wifi_utils.get_connection_info(content)
net_mask = connection_info["net_mask"]
gateway= connection_info["Gateway"]
# find a free IP address
static_ip = wifi_generic_steps.find_available_ip(serial = serial,ip_range=static_ip_range)()

# clear saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# add the Wi-Fi network with static IP
wifi_generic_steps.add_network(serial = serial,
                               ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               ip_settings = "Static",
                               ip_address = static_ip,
                               gateway = gateway,
                               network_prefix_length = net_mask)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED')()
wifi_generic_steps.ping_gateway(serial = serial)()

# forget the SSID
wifi_generic_steps.remove_network(ap_name = ddwrt_ap_name,
                                  serial = serial)()

##### test end #####
