#!/usr/bin/env python

#######################################################################
#
# @description: DUT will  handle the sending process normally when
#               retry after sending failed
# @note:        pushing the picture done with ACS the TestStep
#               INSTALL_FILE; the path and name given as parameters
# @author:      adrian.palko@intel.com
# @author:      mihaela.maracine@intel.com
#
#######################################################################

import sys

from testlib.scripts.wireless.bluetooth import bluetooth_steps
from testlib.base.base_utils import get_args

bluetooth_steps.LogInfo("##### INITIALIZE ######")()

# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
if not script_args:
    raise Exception("--script-args is mandatory")
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory param
if "serial2" not in args.keys() or "photo_folder_name" not in args.keys() or "photo_name" not in args.keys():
    raise Exception("serial2/photo_folder_name/photo_name parameters are mandatory")
serial_dev = args["serial2"]
PHOTO_FOLDER = args['photo_folder_name']
PHOTO_NAME = args['photo_name']

# Initialize versions and names (we only need dev name)
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev, blocking=True)()
PAIRING_DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev, blocking=True)()

# check Photos installed
bluetooth_steps.CheckPackageInstalled(serial=serial, package_name="com.google.android.apps.photos", blocking=True)()

try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    # DUT: clear all notifications etc
    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.ClearDataPackage(serial=serial, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.StopPackage(serial=serial, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.ClearAllNotifications(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial, version=DUT_VERSION, blocking=True)()

    # DEV: clear all notifications/opp files etc
    bluetooth_steps.StopPackage(serial=serial_dev, blocking=True)()
    bluetooth_steps.PressHome(serial=serial_dev, blocking=True)()
    bluetooth_steps.ClearAllNotifications(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.ClearPath(serial=serial_dev, blocking=True)()

    # DUT: turn on bt
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, blocking=True)()

    # DEV: turn on bt
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, blocking=True)()

    # DUT: wait scan and remove paired devices
    bluetooth_steps.WaitBtScanning(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # DEV:  wait scan (should be already finished), rename and remove paired devices
    bluetooth_steps.WaitBtScanning(serial=serial_dev, timeout_appear=0, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial_dev,
                                       name=PAIRING_DEV_NAME, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # send photo via opp, it will be declined
    bluetooth_steps.NavigateToFileInPhotos(serial=serial, local_folder_name=PHOTO_FOLDER, open_video=False,
                                           bypass_tutorial=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtOppSharePhotosFile(serial=serial,
                                         server_dut=str(PAIRING_DEV_NAME),
                                         bt_already_opened=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtOppReceiveFile(serial=serial_dev,
                                     action='Decline', filename_starting_string=PHOTO_NAME, version=DEV_VERSION,
                                     blocking=True)()
    bluetooth_steps.DismissTransferList(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    bluetooth_steps.StopPackage(serial=serial, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()

    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # ############### Test ####################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    # retry sending a failed file; reference device will accept it

    bluetooth_steps.BtOppResendFile(serial=serial, file_to_resend=PHOTO_NAME, version=DUT_VERSION)()
    bluetooth_steps.BtOppReceiveFile(serial=serial_dev,
                                     action='Accept', filename_starting_string=PHOTO_NAME, version=DEV_VERSION)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # DUT: close inbounds transfers, turn off bt, clear all opp notifications/files
    bluetooth_steps.DismissTransferList(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.ClearDataPackage(serial=serial, package_name="com.google.android.apps.photos", critical=False)()
    bluetooth_steps.StopPackage(serial=serial, package_name="com.google.android.apps.photos", critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.ClearAllNotifications(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()

    # DEV: turn off bt, clear all opp notifications/files
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
    bluetooth_steps.ClearAllNotifications(serial=serial_dev, version=DEV_VERSION, critical=False)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial_dev, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClearPath(serial=serial_dev, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="OFF", version=DEV_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
