#!/usr/bin/env python

#######################################################################
#
# @description: Bluetooth PAN - Disconnect/Disable Profile scenarios
#               for 2 devices
# @usage:       python Bluetooth_PAN_Disconnect_Disable_tests.py
#               -s 0000AAAA --script-args serial2=1111BBBB
#               dut_function=pan action=disconnect action_from=pan
# @note:        Connect to Wifi on NAP and close Wifi on PAN must be
#               made outside the test
# @author:      adrian.palko@intel.com
# @since:       06/30/2015
#
#######################################################################

import sys

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps, bt_utils

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

if "serial2" not in args.keys():
    raise Exception("serial2 parameter is mandatory")
serial_dev = args["serial2"]

# ### optional parameters ###

# default values
dut_function = "pan"
action = "disconnect"
action_from = "pan"

# possible values for optional parameters
action_values = ["disconnect", "disable"]
function_values = ["pan", "nap"]

# parse optional parameters
if "dut_function" in args.keys():
    if args["dut_function"].lower() in function_values:
        dut_function = args["dut_function"].lower()
    else:
        raise Exception("Possible values for dut_function: " + str(function_values))
if "action" in args.keys():
    if args["action"].lower() in action_values:
        action = args["action"].lower()
    else:
        raise Exception("Possible values for action: " + str(action_values))
if "action_from" in args.keys():
    if args["action_from"].lower() in function_values:
        action_from = args["action_from"].lower()
    else:
        raise Exception("Possible values for action_from: " + str(function_values))

# Initialize versions and names
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev, blocking=True)()
DUT_NAME = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()
PAIRING_DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev, blocking=True)()

# instantiate serials based on parameters
if dut_function == "nap":
    nap_serial = serial
    pan_serial = serial_dev
    nap_version = DUT_VERSION
    pan_version = DEV_VERSION
else:
    nap_serial = serial_dev
    pan_serial = serial
    nap_version = DEV_VERSION
    pan_version = DUT_VERSION

try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    if dut_function == "nap":
        nap_name = DUT_NAME
        pan_name = PAIRING_DEV_NAME
    else:
        nap_name = PAIRING_DEV_NAME
        pan_name = DUT_NAME

    # stop settings on both devices
    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.StopPackage(serial=serial_dev, blocking=True)()
    bluetooth_steps.PressHome(serial=serial_dev, blocking=True)()

    # NAP: activate BT tethering
    bluetooth_steps.BtSetTethering(serial=nap_serial, state="ON", tethering_settings_opened=False, version=nap_version,
                                   blocking=True)()

    # DUT: turn on BT
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, blocking=True)()

    # DEV: turn on BT
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, blocking=True)()

    # DUT: wait scan, rename device and remove all paired devices
    bluetooth_steps.WaitBtScanning(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial,
                                       name=DUT_NAME, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # DEV: wait scan (should be already finished), rename device and remove all paired devices
    bluetooth_steps.WaitBtScanning(serial=serial_dev, timeout_appear=0, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial_dev,
                                       name=PAIRING_DEV_NAME, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # pair devices
    bt_utils.bt_pair_devices(serial=serial,
                             dev=serial_dev,
                             dut_name=DUT_NAME,
                             dev_name=PAIRING_DEV_NAME,
                             scan_max_attempts=5, version_dut=DUT_VERSION, version_dev=DEV_VERSION, blocking=True)

    # NAP: check internet connection
    bluetooth_steps.CheckInternetConnection(serial=nap_serial, blocking=True)()
    # PAN: activate internet service
    bluetooth_steps.BtSetService(serial=pan_serial, paired_device_name=nap_name, state=True,
                                 service="Internet access", version=pan_version, blocking=True)()
    bluetooth_steps.WaitForState(serial=pan_serial, device_name=nap_name, connected=True, version=pan_version,
                                 blocking=True)()
    bluetooth_steps.WaitForState(serial=nap_serial, device_name=pan_name, connected=True, version=nap_version,
                                 blocking=True)()
    bluetooth_steps.CheckInternetAccessServiceState(serial=pan_serial, state="ON", max_checks=15, blocking=True,
                                                    version=pan_version)()

    # ############### Test ####################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    if action == "disconnect":
        if action_from == "nap":
            bluetooth_steps.BtDisconnectService(serial=nap_serial, device_name=pan_name, version=nap_version)()
        else:
            bluetooth_steps.BtDisconnectService(serial=pan_serial, device_name=nap_name, version=pan_version)()
    else:
        if action_from == "nap":
            bluetooth_steps.BtSetService(serial=nap_serial, paired_device_name=pan_name, state=False,
                                         service="Internet connection sharing", check_if_already=True,
                                         version=nap_version)()
        else:
            bluetooth_steps.BtSetService(serial=pan_serial, paired_device_name=nap_name, state=False,
                                         service="Internet access", check_if_already=True, version=pan_version)()

    bluetooth_steps.WaitForState(serial=pan_serial, device_name=nap_name, connected=False, version=pan_version)()
    bluetooth_steps.WaitForState(serial=nap_serial, device_name=pan_name, connected=False, version=nap_version)()
    bluetooth_steps.CheckInternetAccessServiceState(serial=pan_serial, state="OFF", version=pan_version)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    #  stop settings on both devices
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()

    # NAP: turn off BT tethering
    bluetooth_steps.BtSetTethering(serial=nap_serial, state="OFF", tethering_settings_opened=False, version=nap_version,
                                   critical=False)()

    # DUT: turn on BT
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, critical=False)()

    # DEV: turn on BT
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, critical=False)()

    # DUT: remove all paired devices and turn off BT
    bluetooth_steps.WaitBtScanning(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()

    # DEV: remove all paired devices and turn off BT
    bluetooth_steps.WaitBtScanning(serial=serial_dev, timeout_appear=0, version=DEV_VERSION, critical=False)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="OFF", version=DEV_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
