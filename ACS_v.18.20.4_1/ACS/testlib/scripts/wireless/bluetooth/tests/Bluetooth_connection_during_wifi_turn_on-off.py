#!/usr/bin/env python

#######################################################################
#
# @description: Pair tests scenarios for 2 devices
# @usage:       python Bluetooth_connection_during_wifi_turn_on-off.py -s 0000AAAA
#               --script-args serial2=1111BBBB action_dut=Pair
#                action_dev=Pair
# @author:      narasimha.rao.rayala@intel.com
# @since:       03/22/2018
#
#######################################################################

import sys

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps, bt_utils
from testlib.scripts.wireless.wifi import wifi_steps

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
action_dut = "Pair"
action_dev = "Pair"
initiator = "dev"
action_initiator_first = True
scan_timeout = 60000
scan_max_attempts = 1
timeout_time = 60000

# possible values for optional parameters
action_values = ["Pair"]
initiator_values = ["dut", "dev"]
true_values = ["true", "t", "1", "yes", "y"]

# parse optional parameters
if "action_dut" in args.keys():
    if args["action_dut"] in action_values:
        action_dut = args["action_dut"]
    else:
        raise Exception("Possible values for action_dut: " + str(action_values))
if "action_dev" in args.keys():
    if args["action_dev"] in action_values:
        action_dev = args["action_dev"]
    else:
        raise Exception("Possible values for action_dev: " + str(action_values))
if "initiator" in args.keys():
    if args["initiator"].lower() in initiator_values:
        initiator = args["initiator"]
    else:
        raise Exception("Possible values for initiator: " + str(initiator_values))
if "action_initiator_first" in args.keys():
    if args["action_initiator_first"].lower() in true_values:
        action_initiator_first = True
    else:
        action_initiator_first = False
if "scan_timeout" in args.keys():
    scan_timeout = int(args["scan_timeout"])
if "scan_max_attempts" in args.keys():
    scan_max_attempts = int(args["scan_max_attempts"])
if "timeout_time" in args.keys():
    timeout_time = int(args["timeout_time"])

# Initialize versions and names
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev, blocking=True)()
DUT_NAME = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()
PAIRING_DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev, blocking=True)()

try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    # DUT: turn on BT
    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, blocking=True)()

    # DEV: turn on BT
    bluetooth_steps.StopPackage(serial=serial_dev, blocking=True)()
    bluetooth_steps.PressHome(serial=serial_dev, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, blocking=True)()

    # DUT: wait scanning, rename device and remove all paired devices
    bluetooth_steps.WaitBtScanning(serial=serial, scan_timeout=scan_timeout, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial,
                                       name=DUT_NAME, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # DEV: wait scanning(should be already finished), rename device and remove all paired devices
    bluetooth_steps.WaitBtScanning(serial=serial_dev, timeout_appear=0, scan_timeout=scan_timeout, version=DEV_VERSION,
                                   blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial_dev,
                                       name=PAIRING_DEV_NAME, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # ############ Actual Test ################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    bt_utils.bt_pair_devices(serial=serial, dev=serial_dev,
                             dut_name=DUT_NAME,
                             dev_name=PAIRING_DEV_NAME,
                             action_dut=action_dut,
                             action_dev=action_dev,
                             perform_action_first_on_initiator=action_initiator_first,
                             pair_request_initiator=initiator,
                             scan_timeout=scan_timeout,
                             scan_max_attempts=scan_max_attempts,
                             time_to_wait_timeout_action=timeout_time,
                             version_dut=DUT_VERSION,
                             version_dev=DEV_VERSION)
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    wifi_steps.set_from_wifi_settings(serial=serial,state="OFF")()
    wifi_steps.set_from_wifi_settings(serial=serial,state="ON")()
    wifi_steps.set_from_wifi_settings(serial=serial,state="OFF")()
    if not bluetooth_steps.WaitForState(serial=serial, device_name=PAIRING_DEV_NAME,connected=True, version=DUT_VERSION)():
        raise Exception("Reference Device not connected with DUT")
finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # DUT: stop settings and turn on BT (if not already)
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, critical=False)()

    # DEV: stop settings and turn on BT (if not already)
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, critical=False)()

    # DUT: remove all paired devices and turn off BT
    bluetooth_steps.WaitBtScanning(serial=serial, scan_timeout=scan_timeout, version=DUT_VERSION, critical=False)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()

    # DEV: remove all paired devices and turn off BT
    bluetooth_steps.WaitBtScanning(serial=serial_dev, timeout_appear=0, scan_timeout=scan_timeout, version=DEV_VERSION,
                                   critical=False)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="OFF", version=DEV_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
