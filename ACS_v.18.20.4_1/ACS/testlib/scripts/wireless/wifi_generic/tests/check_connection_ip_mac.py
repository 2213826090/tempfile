#!/usr/bin/env python

######################################################################
#
# @filename:    check_connection_ip_mac.py
# @description: Verifies the IP and MAC after connectin to Wifi AP.
#
# @run example:
#
#            python check_connection_ip_mac.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#                                                       dut_security=wpa
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
from testlib.scripts.wireless.wifi import wifi_utils
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

adb_steps.command(serial = serial,
            command = "settings put global captive_portal_detection_enabled 0",
            mode = "sync",
            timeout = 10)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

conf = wifi_utils.get_connection_content(serial = serial)
conf_dict = wifi_utils.get_connection_info(conf)

# get the MAC address and check there is no IP Address
wifi_generic_steps.check_connection_info(serial = serial,
                                        MAC=conf_dict['MAC'],
                                        ip_address="None")()

# add the Wi-Fi network
wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network and the MAC is the same
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        MAC=conf_dict['MAC'])()

wifi_generic_steps.check_connection_info(serial = serial,
                                        regex=True,
                                        SSID = ddwrt_ap_name,
                                        ip_address=r'[\d\.]+')()

# turn WiFi off
wifi_generic_steps.set_wifi(state = "OFF", serial = serial)()
wifi_generic_steps.check_wifi_state_disconnected(ap_name=ddwrt_ap_name,
                                                 serial = serial)()

# check ip is not present
wifi_generic_steps.check_connection_info(serial = serial,
                                        ip_address="None")()

# turn wifi on
wifi_generic_steps.set_wifi(serial=serial, state = "ON")()
wifi_generic_steps.wait_until_connected(serial = serial)()

# check MAC
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        MAC=conf_dict['MAC'])()

# check IP present
wifi_generic_steps.check_connection_info(serial = serial,
                                        regex=True,
                                        SSID = ddwrt_ap_name,
                                        ip_address=r'[\d\.]+')()

# reboot device
adb_steps.reboot(serial = serial,
                ip_enabled=False)()
adb_steps.root_connect_device(serial = serial)()
ui_steps.set_orientation_vertical(serial = serial, orientation="portrait")()
adb_steps.command(serial = serial,
            command = "settings put global captive_portal_detection_enabled 0",
            mode = "sync",
            timeout = 10)()

# wait until the device connects to wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check MAC
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        MAC=conf_dict['MAC'])()

# check IP present
wifi_generic_steps.check_connection_info(serial = serial,
                                        regex=True,
                                        SSID = ddwrt_ap_name,
                                        ip_address=r'[\d\.]+')()

##### test end #####
