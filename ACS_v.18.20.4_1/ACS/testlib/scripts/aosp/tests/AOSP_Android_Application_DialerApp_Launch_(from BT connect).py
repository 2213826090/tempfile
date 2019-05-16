#!/usr/bin/env python

#######################################################################
#
# @filename:    AOSP_Android_Application_DialerApp_Launch_(from BT connect).py
# @description: Launch Dialer application
# @author:      nidhi.anupama@intel.com
#
#
#
#######################################################################

# Standard libraries
import sys

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.wireless.bluetooth import bluetooth_steps, bt_utils

# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args[0].upper() != 'NONE':
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val


if "serial2" not in args.keys():
    raise Exception("serial2 parameter is mandatory")
serial2 = args["serial2"]


DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial2, blocking=True)()
DUT_NAME = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()
DEV_MAC_ADDRESS = bluetooth_steps.GetBtMac(serial=serial2,
                                           blocking=True)()

# Setup
ui_steps.press_home(serial=serial)()
ui_steps.press_home(serial=serial2)()
ui_steps.am_stop_package(serial=serial, package_name="com.android.settings",
                         blocking=True)()
ui_steps.am_stop_package(serial=serial2, package_name="com.android.settings",
                         blocking=True)()


bluetooth_steps.OpenBluetoothSettings(serial=serial2, use_intent=True,
                                      blocking=True)()
bluetooth_steps.BtChangeDeviceName(serial=serial2, name=DEV_MAC_ADDRESS,
                                   blocking=True)()
bluetooth_steps.OpenBluetoothSettings(serial=serial,
                                      blocking=True)()
bluetooth_steps.BtChangeDeviceName(serial=serial, name=DUT_NAME,
                                   blocking=True)()
bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON",
                                     blocking=True)()

# add step to connect device
bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, blocking=True)()
bluetooth_steps.CheckBtVisibility(serial=serial, blocking=True)()
bluetooth_steps.CheckBtVisibility(serial=serial2, blocking=True)()

# Set Up

bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

bt_utils.bt_pair_devices(serial=serial, dev=serial2,
                             dut_name=DUT_NAME,
                             dev_name=DEV_MAC_ADDRESS,
                             version_dut=DUT_VERSION,
                             version_dev=DEV_VERSION,
                             scan_max_attempts=3)

#Run

ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.press_dialer(serial=serial)()



