#!/usr/bin/env python

#######################################################################
#
# @description: Bluetooth_PAN_BT_Tethering_UI
# @author:      adrian.palko@intel.com
# @author:      lucia.huru@intel.com
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

    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.BtSetTethering(serial=serial, state="OFF", version=DUT_VERSION, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()

    # ############ Actual Test ################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    # ##### Tethering should not turn off when leaving Tethering menu #####
    bluetooth_steps.BtSetTethering(serial=serial, state="ON", check_if_already=True, version=DUT_VERSION)()
    bluetooth_steps.PressHome(serial=serial)()
    bluetooth_steps.BtCheckTetheringState(serial=serial, state="ON", tethering_settings_opened=False,
                                          version=DUT_VERSION)()
    bluetooth_steps.PressHome(serial=serial)()
    # ##### Tethering should be turned off and remain off when BT is ON #####
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", check_if_already=True, version=DUT_VERSION)()
    bluetooth_steps.PressHome(serial=serial)()
    bluetooth_steps.BtCheckTetheringState(serial=serial, state="OFF", tethering_settings_opened=False,
                                          version=DUT_VERSION)()
    bluetooth_steps.PressHome(serial=serial)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", check_if_already=True, version=DUT_VERSION)()
    bluetooth_steps.PressHome(serial=serial)()
    bluetooth_steps.BtCheckTetheringState(serial=serial, state="OFF", tethering_settings_opened=False,
                                          version=DUT_VERSION)()
    bluetooth_steps.PressHome(serial=serial)()
    # ##### Open Tethering settings from recent apps and check BT tethering #####
    bluetooth_steps.BtOpenFromRecent(serial=serial, version=DUT_VERSION)()
    bluetooth_steps.BtCheckTetheringState(serial=serial, state="OFF", tethering_settings_opened=True,
                                          version=DUT_VERSION)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.BtSetTethering(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
