#!/usr/bin/env python

#######################################################################
#
# @filename:    aosp_turn_on_bt_check_available_device.py
# @description: Turn on BT and check the available devices
# @author:      saddam.hussain.abbas@intel.com
# @run example:
#
#       python aosp_turn_on_bt_check_available_device.py -s DutSerialNumber
#               --script-args serial2=DevSerialNumber
#######################################################################

# Build in libraries
import sys

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps
from testlib.scripts.android.ui import ui_steps

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# Mandatory param
if "serial2" not in args.keys():
    raise Exception("serial2 parameter is mandatory")
serial2 = args["serial2"]
# Setup

DEV_MAC_ADDRESS = bluetooth_steps.GetBtMac(serial=serial2,
                                           blocking=True)()

# Setup
ui_steps.press_home(serial=serial)()
ui_steps.press_home(serial=serial2)()
ui_steps.am_stop_package(serial=serial, package_name="com.android.settings",
                         blocking=True)()
ui_steps.am_stop_package(serial=serial2, package_name="com.android.settings",
                         blocking=True)()

# Run
bluetooth_steps.OpenBluetoothSettings(serial=serial2, use_intent=True,
                                      blocking=True)()
bluetooth_steps.BtChangeDeviceName(serial=serial2, name=DEV_MAC_ADDRESS,
                                   blocking=True)()
bluetooth_steps.OpenBluetoothSettings(serial=serial,
                                      blocking=True)()
bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON",
                                     blocking=True)()
# porting needs to be done for the below api to work on Andorid O, currently
#  not done because of the Bt available device refresh
bluetooth_steps.BtSearchDevices(serial=serial,
                                dev_to_find=DEV_MAC_ADDRESS)()

# Teardown
ui_steps.am_stop_package(serial=serial, package_name="com.android.settings",
                         blocking=False)()
ui_steps.am_stop_package(serial=serial2,
                         package_name="com.android.settings", blocking=False)()