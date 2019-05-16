#!/usr/bin/env python

######################################################################
#
# @filename:    add_network_and_connect.py
# @description: Tests that when correctly adding a network the device will
#               connect to it.
#
# @run example:
#
#            python Connection-to-a-hidden-network.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk/wpa2/wep64/wep128
#                                                       encryption=aes/tkip/open/share
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#                                                       dut_security=wpa/wep
#                                                       group_cipher=WEP-40 / WEP-104
#                                                       pairwise_cipher=WEP-40 / WEP-104
#                                                       ap_module=ddwrt_atheros - optional ; defaults to ddwrt_broadcom
#
#           for check_connection_info use the following:
#
#       WEP:
#            if security == "wep64":
#                group_cipherX = "WEP-40"
#                pairwise_cipherX = "WEP-40"
#            else:
#                group_cipherX = "WEP-104"
#                pairwise_cipherX = "WEP-104"
#
#       WPA:
#        if encryption == "aes":
#            group_cipherX = "CCMP"
#            pairwise_cipherX = "CCMP"
#        elif encryption == "tkip":
#            group_cipherX = "TKIP"
#            pairwise_cipherX = "TKIP"
#        else:
#            group_cipherX = "TKIP"
#            pairwise_cipherX = "CCMP"
#
#        if security == "wpa_psk_mixed":
#            securityX = 'WPA2-PSK'
#        elif security == "wpa_psk":
#            securityX = 'WPA-PSK'
#        elif security == "wpa2":
#            securityX = 'WPA2-PSK'
#        elif security == "wpa_enterprise":
#            securityX = 'WPA/IEEE 802.1X/EAP'
#        elif security == "wpa2_enterprise":
#            securityX = 'WPA2/IEEE 802.1X/EAP'

# @author:      stefanx.todirica@intel.com
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
encryption = args["encryption"]
ddwrt_ap_name =  args["ap_name"]
ddwrt_ap_pass = args["passphrase"]
group_cipher = args["group_cipher"]
pairwise_cipher = args["pairwise_cipher"]

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
security_check = None
security_check_alt = None
if "security_check" in args.keys():
    security_check = args["security_check"]
    security_check_alt = security_check.replace("-", "_", 1)

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

# configure hidden ap
ap_steps.setup(mode, security,
               new_ssid = ddwrt_ap_name,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               serial = serial,
               hidden_ssid = "1")()

#wait for the SSID to disappear from AP list
#wifi_generic_steps.scan_and_check_ap(ddwrt_ap_name, trycount=30, should_exist=False, serial = serial)()

# add the hidden Wi-Fi network
wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               serial = serial)()

# wait until the device connects to the wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network
if security_check:
    wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED',
                                        Security=security_check_alt or security_check,
                                        group_cipher=group_cipher,
                                        pairwise_cipher=pairwise_cipher)()
else:
    wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID = ddwrt_ap_name,
                                        state='CONNECTED/CONNECTED',
                                        group_cipher=group_cipher,
                                        pairwise_cipher=pairwise_cipher)()
##### test end #####
