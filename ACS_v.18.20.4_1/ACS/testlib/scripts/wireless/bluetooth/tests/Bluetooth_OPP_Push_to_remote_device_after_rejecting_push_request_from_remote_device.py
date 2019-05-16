#!/usr/bin/env python

#######################################################################
#
# @description: reference(serial_dev) sends file and serial rejects
#               the request. After rejecting, serial also sends a file
#               that serial_dev accepts
# @note:        pushing the picture done with ACS the TestStep
#               INSTALL_FILE; the path and name given as parameters
# @author:      adrian.palko@intel.com
# @author:      mihaela.maracine@intel.com
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

# mandatory param
if "serial2" not in args.keys() or "photo_folder_name" not in args.keys() or \
                "photo_folder_name2" not in args.keys() or "photo_name" not in args.keys() or \
                "photo_name2" not in args.keys():
    raise Exception("serial2/photo_folder_name/photo_folder_name2/photo_name/photo_name2 parameters are mandatory")
serial_dev = args["serial2"]
PHOTO_PATH_DUT = args['photo_folder_name']
PHOTO_PATH_DEV = args['photo_folder_name2']
PHOTO_NAME = args['photo_name']
PHOTO_NAME2 = args['photo_name2']

# Initialize versions and names
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev, blocking=True)()
DUT_NAME = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()
PAIRING_DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev, blocking=True)()

# check Photos installed
bluetooth_steps.CheckPackageInstalled(serial=serial, package_name="com.google.android.apps.photos", blocking=True)()
bluetooth_steps.CheckPackageInstalled(serial=serial_dev, package_name="com.google.android.apps.photos", blocking=True)()

try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    # DUT: clear all notifications/opp files etc
    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.ClearDataPackage(serial=serial, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.StopPackage(serial=serial, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.ClearAllNotifications(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClearPath(serial=serial, blocking=True)()

    # DEV: clear all notifications/opp files etc
    bluetooth_steps.StopPackage(serial=serial_dev, blocking=True)()
    bluetooth_steps.ClearDataPackage(serial=serial_dev, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.StopPackage(serial=serial_dev, package_name="com.google.android.apps.photos", blocking=True)()
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

    # DUT: wait scanning. rename and remove paired devices
    bluetooth_steps.WaitBtScanning(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial,
                                       name=DUT_NAME, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # DEV: wait scanning (should be already finished), rename and remove paired devices
    bluetooth_steps.WaitBtScanning(serial=serial_dev, timeout_appear=0, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial_dev,
                                       name=PAIRING_DEV_NAME, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # first we check if an OPP transfer count has been initiated in the Notification panel
    count_unsuccessful_sent_before = bt_utils.get_sent_received_count(serial=serial_dev,
                                                                      share_type='Sent',
                                                                      count_successful=False,
                                                                      count_unsuccessful=True,
                                                                      notif_menu_already_opened=False,
                                                                      version=DEV_VERSION,
                                                                      blocking=True)

    count_unsuccessful_received_before = bt_utils.get_sent_received_count(serial=serial,
                                                                          share_type='Received',
                                                                          count_successful=False,
                                                                          count_unsuccessful=True,
                                                                          notif_menu_already_opened=False,
                                                                          version=DUT_VERSION,
                                                                          blocking=True)

    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # ############### Test ####################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    # reject incoming transfer

    bluetooth_steps.NavigateToFileInPhotos(serial=serial_dev, local_folder_name=PHOTO_PATH_DEV, open_video=False,
                                           bypass_tutorial=True, version=DEV_VERSION)()
    bluetooth_steps.BtOppSharePhotosFile(serial=serial_dev,
                                         server_dut=str(DUT_NAME),
                                         bt_already_opened=True, version=DEV_VERSION)()
    bluetooth_steps.BtOppReceiveFile(serial=serial,
                                     action='Decline', filename_starting_string=PHOTO_NAME2, version=DUT_VERSION)()

    count_unsuccessful_sent_after = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial_dev,
                                                         share_type='Sent',
                                                         count_successful=False,
                                                         count_unsuccessful=True,
                                                         notif_menu_already_opened=False, version=DEV_VERSION)()
    # compare unsuccessful transfer values on DEV
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial_dev, before_value=count_unsuccessful_sent_before,
                                                   after_value=count_unsuccessful_sent_after,
                                                   increment=1, no_log=True)()

    count_unsuccessful_received_after = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Received',
                                                         count_successful=False,
                                                         count_unsuccessful=True,
                                                         notif_menu_already_opened=False, version=DUT_VERSION)()
    # compare unsuccessful transfer values on DUT
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_unsuccessful_received_before,
                                                   after_value=count_unsuccessful_received_after,
                                                   increment=1, no_log=True)()

    # send file after rejecting incoming transfer; reference device will accept it

    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION)()
    bluetooth_steps.WaitBtScanning(serial=serial_dev, version=DEV_VERSION)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION)()

    bluetooth_steps.NavigateToFileInPhotos(serial=serial, local_folder_name=PHOTO_PATH_DUT, open_video=False,
                                           bypass_tutorial=True, version=DUT_VERSION)()
    bluetooth_steps.BtOppSharePhotosFile(serial=serial,
                                         server_dut=str(PAIRING_DEV_NAME),
                                         bt_already_opened=True, version=DUT_VERSION)()
    bluetooth_steps.BtOppReceiveFile(serial=serial_dev,
                                     action='Accept', filename_starting_string=PHOTO_NAME, version=DEV_VERSION)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # DUT: turn off bt, clear all opp notifications/files
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.ClearDataPackage(serial=serial, package_name="com.google.android.apps.photos", critical=False)()
    bluetooth_steps.StopPackage(serial=serial, package_name="com.google.android.apps.photos", critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.ClearAllNotifications(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClearPath(serial=serial, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()

    # DEV: turn off bt, clear all opp notifications/files
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.ClearDataPackage(serial=serial_dev, package_name="com.google.android.apps.photos",
                                     critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, package_name="com.google.android.apps.photos", critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
    bluetooth_steps.ClearAllNotifications(serial=serial_dev, version=DEV_VERSION, critical=False)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial_dev, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClearPath(serial=serial_dev, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="OFF", version=DEV_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
