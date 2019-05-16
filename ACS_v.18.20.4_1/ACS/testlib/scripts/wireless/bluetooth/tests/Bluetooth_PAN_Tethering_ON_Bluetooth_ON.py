#!/usr/bin/env python

#######################################################################
#
# @description: Turning on Bluetooth Tethering should automatically turn
#               on Bluetooth
# @author:      adrian.palko@intel.com
# @since:       06/25/2015
#
#######################################################################

import sys

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps

bluetooth_steps.LogInfo("##### INITIALIZE ######")()

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))

# Initialize version
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()

try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    # turn off tethering and BT
    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.BtSetTethering(serial=serial, state="OFF", check_if_already=False, version=DUT_VERSION,
                                   blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", check_if_already=False, version=DUT_VERSION,
                                         blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()

    # ############ Actual Test ################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    # enable bt tethering
    bluetooth_steps.BtSetTethering(serial=serial, state="ON", check_if_already=True, version=DUT_VERSION)()
    # disable bt and fail if not already enabled
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", check_if_already=True, version=DUT_VERSION)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # turn off tethering and BT
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.BtSetTethering(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
