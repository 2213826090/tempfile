#!/usr/bin/env python

#######################################################################
#
# @filename:    Bluetooth_GeneralTest_ON_After_Device_Sleep.py
# @description: Tests if Bluetooth ON after device resumed from sleep mode
#
# @author:      narasimha.rao.rayala@intel.com
#
#######################################################################

import sys
import time
from testlib.scripts.wireless.bluetooth import bluetooth_utils
from testlib.scripts.wireless.bluetooth import bluetooth_steps
from testlib.base.base_utils import get_args
from testlib.base import base_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.security.scripts import prerequisites

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
if not script_args:
    raise Exception("--script-args is mandatory")
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
bluetooth_steps.StopPackage(serial=serial, blocking=True)()
bluetooth_steps.PressHome(serial=serial, blocking=True)()
bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, blocking=True)()
###############################################################
# Bluetooth state after device into sleep and wakeup
#sleep and wakeup device through keyevent26 
###############################################################
adb_steps.put_device_into_sleep_mode(serial = serial)()
time.sleep(30)
adb_steps.wake_up_device(serial = serial)()
ui_steps.unlock_device_swipe(serial = serial)()
bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
###checking_BT_Status
bt_current_state = bluetooth_utils.check_bluetooth_state_on(serial)
if bt_current_state is True:
    print "BT is ON after device wakeup from sleep"
###############################################################
# Take the system to the initial status
###############################################################
bluetooth_steps.StopPackage(serial=serial, blocking=True)()
bluetooth_steps.PressHome(serial=serial, blocking=True)()


