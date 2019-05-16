#!/usr/bin/env python

#######################################################################
#
# @description: DUT does not answer to a received file notification
#               from Reference phone
# @note:        pushing the picture done with ACS the TestStep
#               INSTALL_FILE; the path and name given as parameters
# @author:      adrian.palko@intel.com
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
if "serial2" not in args.keys() or "photo_folder_name" not in args.keys() or "photo_name" not in args.keys():
    raise Exception("serial2/photo_folder_name/photo_name parameters are mandatory")
serial_dev = args["serial2"]
PHOTO_PATH = args['photo_folder_name']
PHOTO_NAME = args['photo_name']

# Initialize versions and names
DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial, blocking=True)()
DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev, blocking=True)()
DUT_NAME = bluetooth_steps.GetBtMac(serial=serial, blocking=True)()

# check Photos installed
bluetooth_steps.CheckPackageInstalled(serial=serial_dev, package_name="com.google.android.apps.photos", blocking=True)()

try:

    # ########### Preconditions ###############
    # #########################################

    bluetooth_steps.LogInfo("######## SETUP ########")()

    # DUT: clear all notifications/opp files etc
    bluetooth_steps.StopPackage(serial=serial, blocking=True)()
    bluetooth_steps.PressHome(serial=serial, blocking=True)()
    bluetooth_steps.ClearAllNotifications(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClearPath(serial=serial, blocking=True)()

    # DEV: clear all notifications etc
    bluetooth_steps.StopPackage(serial=serial_dev, blocking=True)()
    bluetooth_steps.ClearDataPackage(serial=serial_dev, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.StopPackage(serial=serial_dev, package_name="com.google.android.apps.photos", blocking=True)()
    bluetooth_steps.PressHome(serial=serial_dev, blocking=True)()
    bluetooth_steps.ClearAllNotifications(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # DUT: turn on bt
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON", version=DUT_VERSION, blocking=True)()

    # DEV: turn on bt
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="ON", version=DEV_VERSION, blocking=True)()

    # DUT:wait scanning, rename and remove paired devices
    bluetooth_steps.WaitBtScanning(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtChangeDeviceName(serial=serial,
                                       name=DUT_NAME, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial, version=DUT_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # setup on DEV: wait scanning (should be already finished) and remove paired devices
    bluetooth_steps.WaitBtScanning(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev, version=DEV_VERSION, blocking=True)()
    bluetooth_steps.CheckBtVisibility(serial=serial_dev, version=DEV_VERSION, blocking=True)()

    # DUT: first we check if an OPP transfer count has been initiated in the Notification panel
    bluetooth_steps.OpenNotificationsMenu(serial=serial, version=DUT_VERSION, blocking=True)()
    count_unsuccessful_received_initial = bt_utils.get_sent_received_count(serial=serial,
                                                                           share_type='Received',
                                                                           count_successful=False,
                                                                           count_unsuccessful=True,
                                                                           notif_menu_already_opened=True,
                                                                           version=DUT_VERSION,
                                                                           blocking=True)
    count_successful_received_initial = bt_utils.get_sent_received_count(serial=serial,
                                                                         share_type='Received',
                                                                         count_successful=True,
                                                                         count_unsuccessful=False,
                                                                         notif_menu_already_opened=True,
                                                                         version=DUT_VERSION,
                                                                         blocking=True)
    bluetooth_steps.CloseNotificationsMenu(serial=serial, version=DUT_VERSION, blocking=True)()

    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION, blocking=True)()

    # ############### Test ####################
    # #########################################

    bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

    # send a file from dev to dut

    bluetooth_steps.NavigateToFileInPhotos(serial=serial_dev, local_folder_name=PHOTO_PATH, open_video=False,
                                           bypass_tutorial=True, version=DEV_VERSION)()
    bluetooth_steps.BtOppSharePhotosFile(serial=serial_dev,
                                         server_dut=str(DUT_NAME),
                                         bt_already_opened=True, version=DEV_VERSION)()

    # the transfer request is left unanswered
    bluetooth_steps.OpenNotificationsMenu(serial=serial, version=DUT_VERSION)()

    if DUT_VERSION.startswith("5.") or DUT_VERSION.startswith("6.0"):
        # LLP, M versions
        notification = "Bluetooth share: Incoming file"
    else:
        # N version
        notification = "Incoming file"

    bluetooth_steps.BtCheckNotificationAppear(serial=serial, text_contains=notification,
                                              click_on_notification=False, time_to_appear=30000, version=DUT_VERSION)()
    bluetooth_steps.BtCheckNotificationGone(serial=serial, text_contains=notification,
                                            time_to_wait=120000, version=DUT_VERSION)()

    bluetooth_steps.OpenNotificationsMenu(serial=serial_dev, version=DEV_VERSION)()
    bluetooth_steps.BtCheckNotificationGone(serial=serial_dev, text_contains='Bluetooth share: Sending ',
                                            time_to_wait=0, version=DEV_VERSION)()
    bluetooth_steps.CloseNotificationsMenu(serial=serial_dev, version=DEV_VERSION)()

    # the transfer is identified as unsuccessful in the notification panel (notifications menu is already opened)
    count_unsuccessful_received_timeout = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Received',
                                                         count_successful=False,
                                                         count_unsuccessful=True,
                                                         notif_menu_already_opened=True, version=DUT_VERSION)()
    count_successful_received_timeout = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Received',
                                                         count_successful=True,
                                                         count_unsuccessful=False,
                                                         notif_menu_already_opened=True, version=DUT_VERSION)()
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_unsuccessful_received_initial,
                                                   after_value=count_unsuccessful_received_timeout,
                                                   increment=1)()
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_successful_received_initial,
                                                   after_value=count_successful_received_timeout,
                                                   increment=0)()
    # and canceled/rejected by user in the Inbound transfers window
    bluetooth_steps.BtCheckNotificationAppear(serial=serial, text_contains="Bluetooth share: Received files",
                                              click_on_notification=True, time_to_appear=0, version=DUT_VERSION)()
    bluetooth_steps.BtOppCheckStatusInTransfersList(serial=serial, file_to_check=PHOTO_NAME,
                                                    exp_status="Transfer canceled by user.", full_status=True,
                                                    transfer_type="Inbound", version=DUT_VERSION)()
    bluetooth_steps.DismissTransferList(serial=serial, fail_if_not_found=True, version=DUT_VERSION)()

    bluetooth_steps.StopPackage(serial=serial_dev, package_name="com.google.android.apps.photos")()

    # send a file from dev to dut
    bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION)()

    bluetooth_steps.NavigateToFileInPhotos(serial=serial_dev, local_folder_name=PHOTO_PATH, open_video=False,
                                           bypass_tutorial=False, version=DEV_VERSION)()
    bluetooth_steps.BtOppSharePhotosFile(serial=serial_dev,
                                         server_dut=str(DUT_NAME),
                                         bt_already_opened=True, version=DEV_VERSION)()
    bluetooth_steps.BtOppReceiveFile(serial=serial,
                                     action='Accept', filename_starting_string=PHOTO_NAME, version=DUT_VERSION)()

    bluetooth_steps.OpenNotificationsMenu(serial=serial, version=DUT_VERSION)()
    count_unsuccessful_received_accept = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Received',
                                                         count_successful=False,
                                                         count_unsuccessful=True,
                                                         notif_menu_already_opened=True, version=DUT_VERSION)()
    count_successful_received_accept = \
        bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=serial,
                                                         share_type='Received',
                                                         count_successful=True,
                                                         count_unsuccessful=False,
                                                         notif_menu_already_opened=True, version=DUT_VERSION)()
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_unsuccessful_received_timeout,
                                                   after_value=count_unsuccessful_received_accept,
                                                   increment=0)()
    bluetooth_steps.BtOppNotificationUpdateCompare(serial=serial, before_value=count_successful_received_timeout,
                                                   after_value=count_successful_received_accept,
                                                   increment=1)()
    bluetooth_steps.BtCheckNotificationAppear(serial=serial, text_contains="Bluetooth share: Received files",
                                              click_on_notification=True, time_to_appear=0, version=DUT_VERSION)()
    bluetooth_steps.BtOppCheckStatusInTransfersList(serial=serial, file_to_check=PHOTO_NAME,
                                                    exp_status="Received complete.", full_status=False,
                                                    transfer_type="Inbound", version=DUT_VERSION)()
    bluetooth_steps.DismissTransferList(serial=serial, fail_if_not_found=True, version=DUT_VERSION)()

finally:

    # ########### Postconditions ##############
    # #########################################

    bluetooth_steps.LogInfo("####### CLEANUP #######")()

    # DUT: turn off bt, clear all opp notifications/files
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()
    bluetooth_steps.ClearAllNotifications(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.BtOppDismissEventualIncoming(serial=serial, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClearPath(serial=serial, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True, version=DUT_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF", version=DUT_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial, critical=False)()
    bluetooth_steps.PressHome(serial=serial, critical=False)()

    # DEV: turn off bt, clear all opp notifications
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.ClearDataPackage(serial=serial_dev, package_name="com.google.android.apps.photos", critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, package_name="com.google.android.apps.photos", critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
    bluetooth_steps.ClearAllNotifications(serial=serial_dev, version=DEV_VERSION, critical=False)()
    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev, use_intent=True, version=DEV_VERSION, critical=False)()
    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev, state="OFF", version=DEV_VERSION, critical=False)()
    bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
    bluetooth_steps.PressHome(serial=serial_dev, critical=False)()
