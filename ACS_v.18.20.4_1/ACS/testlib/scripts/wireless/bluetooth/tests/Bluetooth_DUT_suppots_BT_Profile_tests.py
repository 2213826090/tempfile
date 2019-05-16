#!/usr/bin/env python

#######################################################################
#
# @description: Check that DUT supports BT Profiles
# @note:        For running this test user requires a BT dongle plugged
#               in the linux station that has the devices attached
# @author:      adrian.palko@intel.com
#
#######################################################################

import sys

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps

bluetooth_steps.LogInfo("##### INITIALIZE ######")()

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
if not script_args:
    raise Exception("--script-args is mandatory")
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# ### mandatory parameters ###

if "profile" not in args.keys():
    raise Exception("profile parameter is mandatory")
if "version_value" not in args.keys():
    raise Exception("version_value is mandatory")
profile = args["profile"]
version_value = args["version_value"]

### Redefine the N version "OBEX Object Push" element version_value ###
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
if profile == 'OBEX Object Push' and DUT_VERSION.startswith("7"):
   version_value = '0x0100'

# ### optional parameters ###

profile2 = None
version_value2 = None
if ("profile2" in args.keys() and "version_value2" not in args.keys()) or (
                "profile2" not in args.keys() and "version_value2" in args.keys()):
    raise Exception("both profile2 and version_value2 params must be provided")
elif "profile2" in args.keys():
    profile2 = args["profile2"]
    version_value2 = args["version_value2"]

# Initialize version and mac
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DUT_MAC = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()

try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()

    # ############ Actual Test ################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION)()
    bluetooth_steps.WaitBtScanning(serial=serial, version=DUT_VERSION)()
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION)()

    bluetooth_steps.BtCheckProfiles(serial=serial, bt_mac_add=DUT_MAC,
                                    bl_profile=profile,
                                    bl_version_value=version_value)()
    # if profile2 and version_value2 params were provided, check that profile also
    if profile2:
        bluetooth_steps.BtCheckProfiles(serial=serial, bt_mac_add=DUT_MAC,
                                        bl_profile=profile2,
                                        bl_version_value=version_value2)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
