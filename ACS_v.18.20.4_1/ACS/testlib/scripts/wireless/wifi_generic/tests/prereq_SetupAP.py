#!/usr/bin/env python

######################################################################
#
# @filename:    prereq_SetupAP.py
# @description: Prerequisite test that configures the AP
#
#
# @run example:
#
#            python prereq_SetupAP.py -s 0BA8F2A0
#                                               --script-args
#                                                       mode=n
#                                                       security=wpa_psk
#                                                       encryption=aes
#                                                       ap_name=ddwrt
#                                                       passphrase=test1234
#
# @author:      vlad.a.gruia@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
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
ddwrt_ap_name =  args["ap_name"]

# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi
encryption = wifi_defaults.wifi["encryption"]
if "encryption" in args.keys():
    encryption = args["encryption"]
ddwrt_ap_pass = wifi_defaults.wifi["passphrase"]
if "passphrase" in args.keys():
    ddwrt_ap_pass = args["passphrase"]

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# configure ap
ap_steps.setup(mode, security,
               new_ssid = ddwrt_ap_name,
               encryption = encryption,
               wifi_password = ddwrt_ap_pass,
               serial = serial)()

##### test end #####
