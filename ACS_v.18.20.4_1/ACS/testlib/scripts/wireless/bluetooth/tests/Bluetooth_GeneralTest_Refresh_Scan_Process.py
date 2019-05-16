#!/usr/bin/env python

#######################################################################
#
# @description: Bluetooth Refresh Scan process
# @author:      narasimha.rao.rayala@intel.com
#
#######################################################################

import sys

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps
from uiautomator import Device

bluetooth_steps.LogInfo("##### INITIALIZE ######")()

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
if not script_args:
    raise Exception("--script-args is mandatory")
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# ### mandatory parameters ###

if "serial2" not in args.keys():
    raise Exception("serial2 parameter is mandatory")
serial_dev = args["serial2"]

# Initialize version
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev, blocking=True)()
PAIRING_DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev, blocking=True)()
d=Device(serial)
attempts=3
try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial_dev, name=PAIRING_DEV_NAME,blocking=True)()
    # ############ Actual Test ################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION)()
    counter=1
    while counter < attempts:
        bluetooth_steps.BtSearchDevices(serial=serial, dev_to_find=PAIRING_DEV_NAME, scan_timeout=60000,
                                        version=DUT_VERSION)()
        d.press.back()
        counter=counter+1
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
