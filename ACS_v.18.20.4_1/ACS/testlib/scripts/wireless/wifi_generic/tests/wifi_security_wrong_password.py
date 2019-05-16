#!/usr/bin/env python

######################################################################
#
# @filename:    wifi_security_wrong_password.py
# @description: Tests the behaviour when the incorrect password is given.
#
# @run example:
#
#            python test1.py -s 0BA8F2A0
#                            --script-args
#                                mode=n
#                                security=wpa_psk
#                                encryption=aes
#                                ap_name=ddwrt
#                                passphrase=test1234
#                                dut_security=wpa
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
phase_2_auth=None
if "phase_2_auth" in args.keys():
    phase_2_auth = args["phase_2_auth"]

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# configure ap
ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               radius_ip = radius_ip,
               radius_secret = radius_secret,
               new_ssid = ddwrt_ap_name,
               serial = serial)()

# clear logcat
adb_steps.command("logcat -c",
                  serial = serial)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure the test SSID is not saved in the known list
wifi_generic_steps.clear_saved_networks(serial = serial)()

# try to connect to the AP with wrong password
wifi_generic_steps.add_network(ssid = ddwrt_ap_name,
                               security = dut_security,
                               password = ddwrt_ap_pass + "_wrong",
                               identity = radius_identity,
                               EAP_method = EAP_method,
                               phase_2_auth = phase_2_auth,
                               serial = serial)()

# verify in the GUI that DUT is not connected
wifi_generic_steps.check_wifi_state_disconnected(ap_name = ddwrt_ap_name,
                                    security=dut_security,
                                    encryption=encryption,
                                    wrong_password=True,
                                    serial = serial)()

# check that the SSID is "remembered" on DUT
wifi_generic_steps.scan_and_check_ap(ap = ddwrt_ap_name,
                                     serial=serial)()
#wifi_generic_steps.wifi_check_SSID_known(ap_name = ddwrt_ap_name,
#                                         serial = serial)()

# remove the SSID from the known list
wifi_generic_steps.clear_saved_networks(serial = serial)()
##### test end #####
