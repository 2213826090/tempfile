#!/usr/bin/env python

#######################################################################
#
# @description: Bluetooth_GeneralTest_BT_on_off_iteratively.
# @author:      narasimha.rao.rayala@intel.com
#
#######################################################################

import sys

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps, bt_utils

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

# ### optional parameters ###

# default values
action_dut = "Pair"
action_dev = "Pair"
initiator = "DUT"
action_initiator_first = True
scan_timeout = 60000
scan_max_attempts = 1
timeout_time = 60000
# possible values for optional parameters
action_values = ["Pair","Scan"]
initiator_values = ["dut", "dev"]
true_values = ["true", "t", "1", "yes", "y"]
iteration_values=["1","2","3","4","5","6","7","8"]
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
if "action_initiator" in args.keys():
    if args["action_initiator"] in action_values:
        action_initiator = args["action_initiator"]
    else:
        raise Exception("Possible values for action_initiator: " + str(action_values))
if "initiator" in args.keys():
    if args["initiator"].lower() in initiator_values:
        initiator = args["initiator"]
    else:
        raise Exception("Possible values for initiator: " + str(initiator_values))
if "iteration" in args.keys():
    if args["iteration"] in iteration_values:
        iteration = args["iteration"]
    else:
        raise Exception("Possible values for iteration_values: " + str(iteration_values))

# Initialize version
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

    # ############ Actual Test ################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    counter = 0
    while (counter < int(iteration)):
        if action_initiator == "Pair":
            bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION)()
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
            bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, critical=False)()
            bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, critical=False)()
            bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION)()
        if action_initiator == "Scan":
            bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION)()
            bluetooth_steps.BtSearchDevices(serial=serial_dev, dev_to_find=DUT_NAME, scan_timeout=60000, version = DEV_VERSION)()
            bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION)()
        counter=counter+1
finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # DUT: turn on BT if not already
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()

    # DEV: turn on BT if not already
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="OFF", version=DEV_VERSION, critical=False)()