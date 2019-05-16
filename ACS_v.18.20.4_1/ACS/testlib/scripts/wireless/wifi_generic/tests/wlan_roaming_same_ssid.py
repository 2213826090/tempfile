#!/usr/bin/env python

######################################################################
#
# @filename:    wlan_roaming_same_ssid.py
# @description: WLAN roaming, same SSID
#
# @run example:
#
#            python wlan_roaming_same_ssid.py -s 0BA8F2A0
#                                          --script-args
#                                                  security=wpa2
#                                                  encryption=tkip+aes
#                                                  ap_name=ddwrt123
#                                                  passphrase=qwerasdf
#                                                  dut_security=WPA
#                                                  mode=mixed
#                                                  interface5ghz=2
#                                                  scenario=2
#                                                  iterations=1
#
# @author:      aurel.constantin@intel.com
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
mode = args["mode"]
security = args["security"]
dut_security = args["dut_security"]
ap_name =  args["ap_name"]
scenario = args["scenario"]

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
iterations = 1
if "iterations" in args.keys():
    iterations = args["iterations"]
encryption = None
if "encryption" in args.keys():
    encryption = args["encryption"]
ap_pass = None
if "passphrase" in args.keys():
    ap_pass = args["passphrase"]
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
channel_bw = None
if "channel_bw" in args.keys():
    channel_bw = args["channel_bw"]

if interface5ghz != "2":
    ap_name_1 = ap_name + interface5ghz
    ap_name_virt = ap_name_1 + "_virt"
    ap_name_2 = ap_name_1
else:
    ap_name_1 = ap_name + "0"
    ap_name_virt = ap_name + "_virt"
    ap_name_2 = ap_name + "1"


##### test start #####
interface5ghz_ap1 = interface5ghz
interface5ghz_ap2 = interface5ghz
if interface5ghz == "2":
    interface5ghz_ap1 = "0"
    interface5ghz_ap2 = "1"

# configure ap
ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid =  ap_name_1,
               channel_bw = channel_bw,
               interface5ghz = interface5ghz_ap1,
               serial = serial)()
if interface5ghz_ap2 != interface5ghz_ap1:
    ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid =  ap_name_2,
               channel_bw = channel_bw,
               interface5ghz = interface5ghz_ap2,
               serial = serial)()

# disable all virtual interfaces
ap_steps.setup_virtual_interface(enable=False,
               interface5ghz = interface5ghz_ap1,
               virt_if_idx = "0",
               serial = serial)()

if interface5ghz_ap2 != interface5ghz_ap1:
    ap_steps.setup_virtual_interface(enable=False,
               interface5ghz = interface5ghz_ap2,
               virt_if_idx = "0",
               serial = serial)()

# configure virt ap1
ap_steps.setup_virtual_interface(True, security,
               encryption = encryption,
               wifi_password = ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid =  ap_name_virt,
               interface5ghz = interface5ghz_ap1,
               virt_if_idx = "1",
               serial = serial)()

if interface5ghz_ap2 != interface5ghz_ap1:
    virt_if_idx = "1"
else:
    virt_if_idx = "2"

# configure virt ap2
ap_steps.setup_virtual_interface(True, security,
               encryption = encryption,
               wifi_password = ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid =  ap_name_virt,
               interface5ghz = interface5ghz,
               virt_if_idx = virt_if_idx,
               serial = serial)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# AM off
wifi_generic_steps.set_airplane_mode(state = "OFF", serial = serial)()

# config AP2 virt disabled
ap_steps.setup_virtual_interface(enable=False,
               interface5ghz = interface5ghz_ap2,
               virt_if_idx = virt_if_idx,
               serial = serial)()

# add the Wi-Fi network
wifi_generic_steps.add_network(ssid =  ap_name_virt,
                               security = dut_security,
                               password = ap_pass,
                               identity = radius_identity,
                               EAP_method = EAP_method,
                               phase_2_auth = phase_2_auth,
                               user_certificate = user_certificate,
                               serial = serial)()

# wait until the device connects to a wifi network
wifi_generic_steps.wait_until_connected(serial = serial)()

# check we are connected to the correct network.
wifi_generic_steps.check_connection_info(serial = serial,
                                        SSID =  ap_name_virt,
                                        state='CONNECTED/CONNECTED')()

count = 0
while count < int(iterations):
    if int(iterations) > 1:
        print "============================ iteration: {} =======================".format(count+1)
    # config AP1 virt disabled
    ap_steps.setup_virtual_interface(enable=False,
                   interface5ghz = interface5ghz_ap1,
                   virt_if_idx = "1",
                   serial = serial)()

    # verify that DUT is not connected
    wifi_generic_steps.check_wifi_state_disconnected(ap_name = ap_name_virt,
                                        security = 'wpa',
                                        encryption = encryption,
                                        serial = serial)()

    # config AP2 virt ON
    ap_steps.setup_virtual_interface(True, security,
                   encryption = encryption,
                   wifi_password = ap_pass,
                   radius_ip = radius_ip,
                   radius_secret = radius_secret,
                   new_ssid =  ap_name_virt,
                   interface5ghz = interface5ghz_ap2,
                   virt_if_idx = virt_if_idx,
                   serial = serial)()

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = serial)()

    # check we are connected to the correct network.
    wifi_generic_steps.check_connection_info(serial = serial,
                                            SSID = ap_name_virt,
                                            state='CONNECTED/CONNECTED')()

    if scenario == "1":
        # make sure there are no saved networks
        wifi_generic_steps.clear_saved_networks(serial = serial)()

        # verify that DUT is not connected
        wifi_generic_steps.check_wifi_state_disconnected(ap_name = ap_name_virt,
                                            security = 'wpa',
                                            encryption = encryption,
                                            serial = serial)()
    else:
        # config AP2 virt OFF
        ap_steps.setup_virtual_interface(enable=False,
                       interface5ghz = interface5ghz_ap2,
                       virt_if_idx = virt_if_idx,
                       serial = serial)()

        # check disconnected
        wifi_generic_steps.check_wifi_state_disconnected(ap_name = ap_name_virt,
                                            security = 'wpa',
                                            encryption = encryption,
                                            serial = serial)()

        # config AP1 virt ON
        ap_steps.setup_virtual_interface(True, security,
                       encryption = encryption,
                       wifi_password = ap_pass,
                       radius_ip = radius_ip,
                       radius_secret = radius_secret,
                       new_ssid =  ap_name_virt,
                       interface5ghz = interface5ghz_ap1,
                       virt_if_idx = "1",
                       serial = serial)()


        # check connected
        wifi_generic_steps.wait_until_connected(serial = serial)()

        # check we are connected to the correct network.
        wifi_generic_steps.check_connection_info(serial = serial,
                                                SSID = ap_name_virt,
                                                state='CONNECTED/CONNECTED')()
    count += 1
##### test end #####
