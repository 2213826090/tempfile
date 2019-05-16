#!/usr/bin/env python

#######################################################################
#
# @description: after sending 1 file success/fail, notification
#               should change the success/fail number accordingly
#               in notification
# @note:        pushing the picture done with ACS the TestStep
#               INSTALL_FILE; the path and name given as parameters
# @author:      adrian.palko@intel.com
# @since:       12/17/2015
#
#######################################################################

import sys

from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps

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
DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev, blocking=True)()

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
                                       name=DEV_NAME, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # first we initiate an OPP transfer count in the Notification panel
    bluetooth_steps.NavigateToFileInPhotos(serial=serial, local_folder_name=PHOTO_FOLDER, open_video=False,
                                           bypass_tutorial=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtOppSharePhotosFile(serial=serial,
                                         server_dut=str(DEV_NAME),
                                         bt_already_opened=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtOppReceiveFile(serial=serial_dev,
                                     action='Accept', filename_starting_string=PHOTO_NAME, version=DEV_VERSION,
                                     blocking=True)()
    bluetooth_steps.DismissTransferList(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    bluetooth_steps.StopPackage(serial=serial, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()

    bluetooth_steps.OpenNotificationsMenu(serial=serial, version=DUT_VERSION, blocking=True)()
    count_successful_sent_initial = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Sent',
                                                         count_successful=True,
                                                         count_unsuccessful=False,
                                                         notif_menu_already_opened=True, version=DUT_VERSION,
                                                         blocking=True)()

    count_unsuccessful_sent_initial = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Sent',
                                                         count_successful=False,
                                                         count_unsuccessful=True,
                                                         notif_menu_already_opened=True, version=DUT_VERSION,
                                                         blocking=True)()
    bluetooth_steps.CloseNotificationsMenu(serial=serial, version=DUT_VERSION, blocking=True)()

    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # ############### Test ####################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    # send one file to another device via BT successfully. Check the notification.
    bluetooth_steps.NavigateToFileInPhotos(serial=serial, local_folder_name=PHOTO_FOLDER, open_video=False,
                                           bypass_tutorial=False, version=DUT_VERSION)()
    bluetooth_steps.BtOppSharePhotosFile(serial=serial,
                                         server_dut=str(DEV_NAME),
                                         bt_already_opened=True, version=DUT_VERSION)()
    bluetooth_steps.BtOppReceiveFile(serial=serial_dev,
                                     action='Accept', filename_starting_string=PHOTO_NAME, version=DEV_VERSION)()
    bluetooth_steps.DismissTransferList(serial=serial_dev, version=DEV_VERSION)()

    bluetooth_steps.StopPackage(serial=serial, package_name="com.google.android.apps.photos")()
    bluetooth_steps.PressHome(serial=serial)()

    bluetooth_steps.OpenNotificationsMenu(serial=serial, version=DUT_VERSION)()
    count_successful_sent_file_accept = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Sent',
                                                         count_successful=True,
                                                         count_unsuccessful=False,
                                                         notif_menu_already_opened=True, version=DUT_VERSION)()

    count_unsuccessful_sent_file_accept = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Sent',
                                                         count_successful=False,
                                                         count_unsuccessful=True,
                                                         notif_menu_already_opened=True, version=DUT_VERSION)()

    # In notification, the successful send file number should add 1
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_successful_sent_initial,
                                                   after_value=count_successful_sent_file_accept,
                                                   increment=1)()
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_unsuccessful_sent_initial,
                                                   after_value=count_unsuccessful_sent_file_accept,
                                                   increment=0)()

    bluetooth_steps.CloseNotificationsMenu(serial=serial, version=DUT_VERSION)()

    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION)()

    # send one file to another device via BT unsuccessfully. Check the notification.
    bluetooth_steps.NavigateToFileInPhotos(serial=serial, local_folder_name=PHOTO_FOLDER, open_video=False,
                                           bypass_tutorial=False, version=DUT_VERSION)()
    bluetooth_steps.BtOppSharePhotosFile(serial=serial,
                                         server_dut=str(DEV_NAME),
                                         bt_already_opened=True, version=DUT_VERSION)()
    bluetooth_steps.BtOppReceiveFile(serial=serial_dev,
                                     action='Decline', filename_starting_string=PHOTO_NAME, version=DEV_VERSION)()

    bluetooth_steps.OpenNotificationsMenu(serial=serial, version=DUT_VERSION)()
    count_successful_sent_file_decline = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Sent',
                                                         count_successful=True,
                                                         count_unsuccessful=False,
                                                         notif_menu_already_opened=True, version=DUT_VERSION)()

    count_unsuccessful_sent_file_decline = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Sent',
                                                         count_successful=False,
                                                         count_unsuccessful=True,
                                                         notif_menu_already_opened=True, version=DUT_VERSION)()

    # In notification, the unsuccessful send file number should add 1
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_successful_sent_file_accept,
                                                   after_value=count_successful_sent_file_decline,
                                                   increment=0)()
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_unsuccessful_sent_file_accept,
                                                   after_value=count_unsuccessful_sent_file_decline,
                                                   increment=1)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # DUT: turn off bt, clear all opp notifications
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.ClearDataPackage(serial=serial, package_name="com.google.android.apps.photos",
                                     critical=False)()
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
