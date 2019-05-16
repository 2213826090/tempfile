#!/usr/bin/env python

#######################################################################
#
# @description: Connection OF multiple reference devices to IVI.
# @author:      narasimha.rao.rayala@intel.com
#
#######################################################################

import sys

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps, bt_utils
from testlib.scripts.android.ui import ui_steps

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

if "serial3" not in args.keys():
    raise Exception("serial3 parameter is mandatory")
serial_dev3 = args["serial3"]
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
action_values = ["Pair","SyncContactDevice1","SyncContactDevice2","SyncContactTwoDevices"]
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
if "action_initiator" in args.keys():
    if args["action_initiator"] in action_values:
        action_initiator = args["action_initiator"]
    else:
        raise Exception("Possible values for action_initiator: " + str(action_values))
if "action_initiator3" in args.keys():
    if args["action_initiator3"] in action_values:
        action_initiator3 = args["action_initiator3"]
    else:
        raise Exception("Possible values for action_initiator: " + str(action_values))
if "initiator" in args.keys():
    if args["initiator"].lower() in initiator_values:
        initiator = args["initiator"]
    else:
        raise Exception("Possible values for initiator: " + str(initiator_values))

# Initialize version
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev, blocking=True)()
DEV_VERSION3 = bluetooth_steps.GetAndroidVersion(serial=serial_dev3, blocking=True)()
DUT_NAME = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()
PAIRING_DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev, blocking=True)()
PAIRING_DEV_NAME3 = bluetooth_steps.GetBtMac(serial=serial_dev3, blocking=True)()
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
    # DEV3: turn on BT
    bluetooth_steps.StopPackage(serial=serial_dev3, blocking=True)()
    bluetooth_steps.PressHome(serial=serial_dev3, blocking=True)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev3, use_intent=True, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev3, state="ON", version=DEV_VERSION, blocking=True)()

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
    # DEV3: wait scan (should be already finished), rename device and remove all paired devices
    bluetooth_steps.WaitBtScanning(serial=serial_dev3, timeout_appear=0, version=DEV_VERSION3, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial_dev3,
                                       name=PAIRING_DEV_NAME3, version=DEV_VERSION3, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev3, version=DEV_VERSION3, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev3, version=DEV_VERSION3, blocking=True)()

    # ############ Actual Test ################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

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
        bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev3, state="ON", version=DEV_VERSION, blocking=True)()
        bt_utils.bt_pair_devices(serial=serial, dev=serial_dev3,
                                    dut_name=DUT_NAME,
                                    dev_name=PAIRING_DEV_NAME3,
                                    action_dut=action_dut,
                                    action_dev=action_dev,
                                    perform_action_first_on_initiator=action_initiator_first,
                                    pair_request_initiator=initiator,
                                    scan_timeout=scan_timeout,
                                    scan_max_attempts=scan_max_attempts,
                                    time_to_wait_timeout_action=timeout_time,
                                    version_dut=DUT_VERSION,
                                    version_dev=DEV_VERSION3)

    if action_initiator == "SyncContactDevice1":
        bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev3, state="ON", version=DEV_VERSION3, blocking=True)()
        bt_utils.bt_pair_devices(serial=serial, dev=serial_dev3,
                                    dut_name=DUT_NAME,
                                    dev_name=PAIRING_DEV_NAME3,
                                    action_dut=action_dut,
                                    action_dev=action_dev,
                                    perform_action_first_on_initiator=action_initiator_first,
                                    pair_request_initiator=initiator,
                                    scan_timeout=scan_timeout,
                                    scan_max_attempts=scan_max_attempts,
                                    time_to_wait_timeout_action=timeout_time,
                                    version_dut=DUT_VERSION,
                                    version_dev=DEV_VERSION3)
        bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, blocking=True)()
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
        bluetooth_steps.BtSetService(serial=serial_dev,
                                     paired_device_name=DUT_NAME, state=True,
                                     service="Contact sharing", check_if_already=False,
                                     disable_profile_confirm=True,version=DEV_VERSION)()
        bluetooth_steps.BtSetService(serial=serial,
                                     paired_device_name=PAIRING_DEV_NAME, state=False,
                                     service="Contact sharing", check_if_already=True,
                                     disable_profile_confirm=True,version=DUT_VERSION)()
        bluetooth_steps.BtSetService(serial=serial,
                                     paired_device_name=PAIRING_DEV_NAME, state=True,
                                     service="Contact sharing", check_if_already=False,
                                     disable_profile_confirm=True,version=DUT_VERSION)()
        ui_steps.press_car(serial=serial)()
        if not ui_steps.wait_for_view_common(serial=serial, view_to_find={"textContains": "Contacts"})():
            ui_steps.press_car(serial=serial)()
        if not ui_steps.click_button_common(serial=serial, view_to_find={"textContains": "Contacts"},
                                            view_to_check={"className": "android.widget.TextView"})():
            raise Exception("Contacts list not found")

    if action_initiator == "SyncContactDevice2":
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
        bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev3, state="OFF", version=DEV_VERSION3, blocking=True)()
        bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev3, state="ON", version=DEV_VERSION3, blocking=True)()
        bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION)()
        bt_utils.bt_pair_devices(serial=serial, dev=serial_dev3,
                                    dut_name=DUT_NAME,
                                    dev_name=PAIRING_DEV_NAME3,
                                    action_dut=action_dut,
                                    action_dev=action_dev,
                                    perform_action_first_on_initiator=action_initiator_first,
                                    pair_request_initiator=initiator,
                                    scan_timeout=scan_timeout,
                                    scan_max_attempts=scan_max_attempts,
                                    time_to_wait_timeout_action=timeout_time,
                                    version_dut=DUT_VERSION,
                                    version_dev=DEV_VERSION3)
        bluetooth_steps.BtSetService(serial=serial_dev3,
                                     paired_device_name=DUT_NAME, state=True,
                                     service="Contact sharing", check_if_already=False,
                                     disable_profile_confirm=True,version=DEV_VERSION3)()
        bluetooth_steps.BtSetService(serial=serial,
                                     paired_device_name=PAIRING_DEV_NAME3, state=False,
                                     service="Contact sharing", check_if_already=True,
                                     disable_profile_confirm=True,version=DUT_VERSION)()
        bluetooth_steps.BtSetService(serial=serial,
                                     paired_device_name=PAIRING_DEV_NAME3, state=True,
                                     service="Contact sharing", check_if_already=False,
                                     disable_profile_confirm=True,version=DUT_VERSION)()
        ui_steps.press_car(serial=serial)()
        if not ui_steps.wait_for_view_common(serial=serial, view_to_find={"textContains": "Contacts"})():
            ui_steps.press_car(serial=serial)()
        if not ui_steps.click_button_common(serial=serial, view_to_find={"textContains": "Contacts"},
                                            view_to_check={"className": "android.widget.TextView"})():
            raise Exception("Contacts list not found")

    if action_initiator == "SyncContactTwoDevices":
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
        bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev3, state="OFF", version=DEV_VERSION3, blocking=True)()
        bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev3, state="ON", version=DEV_VERSION3, blocking=True)()
        bt_utils.bt_pair_devices(serial=serial, dev=serial_dev3,
                                    dut_name=DUT_NAME,
                                    dev_name=PAIRING_DEV_NAME3,
                                    action_dut=action_dut,
                                    action_dev=action_dev,
                                    perform_action_first_on_initiator=action_initiator_first,
                                    pair_request_initiator=initiator,
                                    scan_timeout=scan_timeout,
                                    scan_max_attempts=scan_max_attempts,
                                    time_to_wait_timeout_action=timeout_time,
                                    version_dut=DUT_VERSION,
                                    version_dev=DEV_VERSION3)
        bluetooth_steps.BtSetService(serial=serial_dev3,
                                     paired_device_name=DUT_NAME, state=True,
                                     service="Contact sharing", check_if_already=False,
                                     disable_profile_confirm=True,version=DEV_VERSION3)()
        bluetooth_steps.BtSetService(serial=serial_dev,
                                     paired_device_name=DUT_NAME, state=True,
                                     service="Contact sharing", check_if_already=False,
                                     disable_profile_confirm=True,version=DEV_VERSION)()
        bluetooth_steps.BtSetService(serial=serial,
                                     paired_device_name=PAIRING_DEV_NAME3, state=False,
                                     service="Contact sharing", check_if_already=True,
                                     disable_profile_confirm=True,version=DUT_VERSION)()
        bluetooth_steps.BtSetService(serial=serial,
                                     paired_device_name=PAIRING_DEV_NAME3, state=True,
                                     service="Contact sharing", check_if_already=False,
                                     disable_profile_confirm=True,version=DUT_VERSION)()
        ui_steps.press_car(serial=serial)()
        if not ui_steps.wait_for_view_common(serial=serial, view_to_find={"textContains": "Contacts"})():
            ui_steps.press_car(serial=serial)()
        if not ui_steps.click_button_common(serial=serial, view_to_find={"textContains": "Contacts"},
                                            view_to_check={"className": "android.widget.TextView"})():
            raise Exception("Contacts list not found")

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # DUT: turn off BT if not already
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()

    # DEV: turn off BT if not already
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="OFF", version=DEV_VERSION, critical=False)()

    # DEV3: turn off BT if not already
    bluetooth_steps.StopPackage(serial=serial_dev3, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev3, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev3, use_intent=True, version=DEV_VERSION3, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev3, state="OFF", version=DEV_VERSION3, critical=False)()