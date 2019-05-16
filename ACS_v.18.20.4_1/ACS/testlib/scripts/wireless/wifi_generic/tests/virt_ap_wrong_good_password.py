#!/usr/bin/env python

######################################################################
#
# @filename:    add_network_and_connect.py
# @description: Tests that when correctly adding a network the device will
#               connect to it.
#
# @run example:
#
#            python add_network_and_connect.py -s 0BA8F2A0
#                                               --script-args
#                                                       test_type=auto_roam / manual_roam
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#                                                       dut_security=wpa
#
# @author:      corneliu.stoicescu@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps

# mandatory params
test_type = args["test_type"]
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
radius_ip = None
if "radius_ip" in args.keys():
    radius_ip = args["radius_ip"]
radius_secret = None
if "radius_secret" in args.keys():
    radius_secret = args["radius_secret"]
radius_identity = None
if "radius_identity" in args.keys():
    radius_identity = args["radius_identity"]
EAP_method=None
if "EAP_method" in args.keys():
    EAP_method = args["EAP_method"]
phase_2_auth = None
if "phase_2_auth" in args.keys():
    phase_2_auth = args["phase_2_auth"]
user_certificate = None
if "user_certificate" in args.keys():
    user_certificate = args["user_certificate"]
interface5ghz = "0"
if "interface5ghz" in args.keys():
    interface5ghz = args["interface5ghz"]
ssh_host = None
if "ssh_host" in args.keys():
    ssh_host = args["ssh_host"]
ssh_user = None
if "ssh_user" in args.keys():
    ssh_user = args["ssh_user"]

##### test start #####

# configure virt ap
ap_steps.setup_virtual_interface(True, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid = ddwrt_ap_name+"_virt",
               interface5ghz = interface5ghz,
               serial = serial)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()


if test_type == "auto_roam":
    # add the physical wifi network
    wifi_generic_steps.add_network(ssid = ddwrt_ap_name+"_virt",
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               identity = radius_identity,
                               EAP_method = EAP_method,
                               phase_2_auth = phase_2_auth,
                               user_certificate = user_certificate,
                               serial = serial)()


    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = serial)()

# clear logcat
adb_steps.command("logcat -c",
                  serial = serial)()

# add the Wi-Fi network
wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = "WPA2",
                               password = "test1234_WORNG",
                               serial = serial)()

# verify that DUT is not connected
wifi_generic_steps.check_wifi_state_disconnected(ap_name = ddwrt_ap_name,
                                    security = 'wpa',
                                    encryption = encryption,
                                    wrong_password = True,
                                    serial = serial)()

if test_type == "auto_roam":
    # wait until the device automatically roams to the remembered wifi network
    wifi_generic_steps.wait_until_connected(serial = serial, timeout=300)()

    # check we are connected to the correct network.
    wifi_generic_steps.check_connection_info(serial = serial,
                                            SSID = ddwrt_ap_name+"_virt",
                                            state='CONNECTED/CONNECTED')()

elif test_type == "manual_roam":
    # add the physical wifi network
    wifi_generic_steps.add_network(ssid = ddwrt_ap_name+"_virt",
                               security = dut_security,
                               password = ddwrt_ap_pass,
                               identity = radius_identity,
                               EAP_method = EAP_method,
                               phase_2_auth = phase_2_auth,
                               user_certificate = user_certificate,
                               serial = serial)()

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = serial)()

    # check we are connected to the correct network.
    wifi_generic_steps.check_connection_info(serial = serial,
                                            SSID = ddwrt_ap_name+"_virt",
                                            state='CONNECTED/CONNECTED')()

##### test end #####
