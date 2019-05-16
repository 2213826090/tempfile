#!/usr/bin/env python

#######################################################################
#
# @description: Pair tests scenarios for 2 devices
# @usage:       Bluetooth_GeneralTest_Pair_non_android_OS.py -s 0000AAAA
#               --script-args bt_name=bt_name_ref action_dut=Pair
#                initiator=dut action_initiator_first=true
#               scan_timeout=60000 scan_max_attempts=1 timeout_time=60000
# @author:      narasimha.rao.rayala@intel.com 
# @since:       09/08/2017
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

if "bt_name" not in args.keys():
    raise Exception("bt_name parameter is mandatory")
bt_name_ref = args["bt_name"]

# ### optional parameters ###
# default values
action_dut = "Pair"
initiator = "DUT"
action_initiator = "DUT"
action_initiator_first = True
scan_timeout = 60000
scan_max_attempts = 1
timeout_time = 60000
timeout = 60000
bt_name_ref = bt_name_ref.replace("_", " ")
##get_args(sys.argv) not identifying script argument with space,added above line, once it is implemented in get_args we will remove above line### 
version_initiator = "DUT_VERSION"
# possible values for optional parameters
action_values = ["Pair","Timeout"]
initiator_values = ["dut"]
true_values = ["true", "t", "1", "yes", "y"]

# parse optional parameters
if "action_dut" in args.keys():
    if args["action_dut"] in action_values:
        action_dut = args["action_dut"]
    else:
        raise Exception("Possible values for action_dut: " + str(action_values))
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
DUT_NAME = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()
try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    # DUT: turn on BT
    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, blocking=True)()

    # DUT: wait scanning, rename device and remove all paired devices
    bluetooth_steps.WaitBtScanning(serial=serial, scan_timeout=scan_timeout, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # ############ Actual Test ################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()
    
    bluetooth_steps.PairDevice(serial=serial, dev_to_pair_name=bt_name_ref,
                                            scan_timeout=scan_timeout, timeout=timeout,
                                            scan_max_attempts=scan_max_attempts)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # DUT: stop settings and turn on BT (if not already)
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, critical=False)()

    # DUT: remove all paired devices and turn off BT
    bluetooth_steps.WaitBtScanning(serial=serial, scan_timeout=scan_timeout, version=DUT_VERSION, critical=False)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()

