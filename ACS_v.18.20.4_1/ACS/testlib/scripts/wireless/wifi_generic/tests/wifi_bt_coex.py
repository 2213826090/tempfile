#!/usr/bin/env python

######################################################################
#
# @filename:
# @description:
#
# @run example:
#
#            python
# @author:      saddam.hussain.abbas@intel.com
#
#######################################################################

##### imports #####
import sys
import time
import os
from multiprocessing import Process, Queue, Condition, active_children, Event
from collections import OrderedDict
import traceback

from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.utils.defaults import wifi_defaults
from testlib.utils.statics.android import statics
from testlib.scripts.wireless.bluetooth import bluetooth_steps
from testlib.scripts.wireless.bluetooth import bluetooth_utils
from testlib.utils import logger
from testlib.base.base_step import BlockingError, FailedError

# protocol=ftp port_number=20211 file_name=generated.bin
# device_path=/storage/sdcard0/Download/ file_size=30000
# mode=bg security=none ap_name=qwe1234 dut_security=None conf_security=NONE
# airplane_mode=ON compare_method=md5
# ap_module=ddwrt_atheros

# constants
VIEW_TIMEOUT = 10000
TRANSFER_TIMEOUT = 200000

CONDITION_EVENT_TIMEOUT = 200
JOIN_TIMEOUT = 1200

log = None
if "LOG_PATH" in os.environ:
    testlib_log_path = os.environ["LOG_PATH"]
else:
    import testlib

    testlib_log_path = os.path.dirname(testlib.__file__) + "/logs/"

log = logger.testlib_log(log_path=testlib_log_path, log_name="testlib_default")

valid_actions = ["FtpDownloadFile", "OppTransferFile", "WifiOnOff", "BtOnOff", "A2dpBtConnect"]

priority = None


def toggle_bt(serial, event, check_wifi_status=True, iteration=1, q=None):
    results = []

    event.wait(CONDITION_EVENT_TIMEOUT)
    event.clear()
    try:
        for _ in range(0, iteration):
            results.append(bluetooth_steps.SetBT(serial=serial, state='OFF')())
            time.sleep(2)
            results.append(bluetooth_steps.SetBT(serial=serial, state='ON')())

            if check_wifi_status:
                results.append(check_power_status(serial, 'wifi'))
    except:
        log.error("BtOnOff: Unexpected error")
        log.error(traceback.format_exc())
        results.append(False)

    if False in results:
        action_result = ['BtOnOff', 'Fail', results]
    else:
        action_result = ['BtOnOff', 'Pass']
    # print 'bt', result
    return action_result if q is None else q.put(action_result)


def toggle_wifi(serial, event, check_bt_status=True, iteration=1, q=None):
    results = []

    event.wait(CONDITION_EVENT_TIMEOUT)
    event.clear()
    try:
        for _ in range(0, iteration):
            results.append(wifi_generic_steps.set_wifi(serial=serial,
                                                       state='OFF')())
            time.sleep(2)
            results.append(wifi_generic_steps.set_wifi(serial=serial,
                                                       state='ON')())

            if check_bt_status:
                results.append(check_power_status(serial, 'bt'))
    except:
        results.append(False)
        log.error("WifiOnOff: Unexpected error")
        log.error(traceback.format_exc())

    if priority == "a2dp":
        event.set()

    if False in results:
        action_result = ['WifiOnOff', 'Fail', results]
    else:
        action_result = ['WifiOnOff', 'Pass']
    # print 'wifi', result
    return action_result if q is None else q.put(action_result)


def check_power_status(serial, for_what, q=None):
    if for_what == 'bt':
        status = bluetooth_utils.check_bluetooth_state_on(serial)
    elif for_what == 'wifi':
        status = wifi_utils.check_wifi_state_on(serial)
    # print 'status', status
    if status is True or status is False:
        return status if q is None else q.put([status])
    else:
        msg = 'Unable to determine ' + for_what + 'status'
        log.error(msg)
        # Todo
        # Need to send resolution as Blokced here and exist from test or
        # recurs to check status to reconfirm


def check_power_status_periodically(serial, for_what, msg_q, time_slab=15,
                                    q=None):
    result = []

    try:
        while msg_q.empty():
            result.append(check_power_status(serial, for_what))
            time.sleep(time_slab)
    except:
        result.append(False)
        log.error("{} power check: Unexpected error".format(
            for_what.capitalize))
        log.error(traceback.format_exc())

    if q is None:
        return result
    else:
        if False in result:
            q.put([for_what+' power stability check', 'Fail', result])
        else:
            q.put([for_what+' power stability check', 'Pass'])

def connect_to_ap(serial, condition, mode='bg', security='none',
                  dut_security=None, ddwrt_ap_name='ddwrt',
                  ddwrt_ap_pass='test1234', encryption=None, iteration=1,
                  q=None):
    for _ in range(0, iteration):
        # configure ap
        ap_steps.setup(mode, security, encryption=encryption,
                       wifi_password=ddwrt_ap_pass, new_ssid=ddwrt_ap_name,
                       serial=serial)()

        # turn display on, if turned off
        ui_steps.wake_up_device(serial=serial)()

        # ensure the device is unlocked
        ui_steps.unlock_device(serial=serial, pin=wifi_defaults.wifi['pin'])()

        # go to home screen
        ui_steps.press_home(serial=serial)()

        # make sure there are no saved networks
        wifi_generic_steps.clear_saved_networks(serial=serial)()

        # add the Wi-Fi network
        wifi_generic_steps.add_network(ssid=ddwrt_ap_name,
                                       security=dut_security,
                                       password=ddwrt_ap_pass, serial=serial)()

        # wait until the device connects to a wifi network
        wifi_generic_steps.wait_until_connected(serial=serial)()

        # check we are connected to the correct network.
        wifi_generic_steps.check_connection_info(serial=serial,
                                                 SSID=ddwrt_ap_name,
                                                 state='CONNECTED/CONNECTED')()

        # check connection
        obj = wifi_generic_steps.ping_gateway(serial=serial)
        obj()
        if 'passed' in obj.resolution.lower():
            return True if q is None else q.put(['Pass'])
        else:
            return False if q is None else q.put(['Success'])


def pair_device(initiator_serial, receiver_serial, initiator_name,
                receiver_name, version_initiator, version_receiver,
                scan_max_attempts=2, timeout=10000, scan_timeout=60000,
                **kwargs):
    try:
        bluetooth_steps.BtSearchDevices(serial=receiver_serial,
            dev_to_find=initiator_name, scan_timeout=scan_timeout, timeout=timeout,
            max_attempts=scan_max_attempts, version=version_receiver, **kwargs)()
        bluetooth_steps.InitiatePairRequest(serial=initiator_serial,
            dev_to_pair_name=receiver_name, scan_timeout=scan_timeout, timeout=timeout,
            scan_max_attempts=scan_max_attempts, version=version_initiator, **kwargs)()
        bluetooth_steps.ReceivePairRequest(serial=receiver_serial,
            dev_receiving_from_name=initiator_name, version=version_receiver,
                                           timeout=timeout, **kwargs)()

        # now we have both devices into pair request window, so we can check the passkey
        passkey_string_initiator = bluetooth_steps.GetPasskey(
            serial=initiator_serial, version=version_initiator, **kwargs)()
        passkey_string_receiver = bluetooth_steps.GetPasskey(
            serial=receiver_serial, version=version_receiver, **kwargs)()
        bluetooth_steps.PasskeyCheck(serial=serial,
            passkey_initiator=passkey_string_initiator,
            passkey_receiver=passkey_string_receiver, **kwargs)()

        # if actions are performed first on pair request initiator, we have the following scenarios that must
        # be treated
        bluetooth_steps.PerformActionPairRequest(serial=initiator_serial,
            action="Pair", version=version_initiator, **kwargs)()
        bluetooth_steps.PerformActionPairRequest( serial=receiver_serial,
            action="Pair", version=version_receiver, **kwargs)()

        bluetooth_steps.CheckIfPaired(serial=initiator_serial,
                                      dev_paired_with=receiver_name,
                                      paired=True, timeout=timeout,
                                      version=version_initiator, **kwargs)()
        bluetooth_steps.CheckIfPaired(serial=receiver_serial,
                                      dev_paired_with=initiator_name,
                                      paired=True, timeout=timeout,
                                      version=version_receiver, **kwargs)()
    except:
        log.error("Failed to pair dut and reference device")
        return False
    return True

def pair_a2dp_device(serial, a2dp_device_name, event, timeout=10000,
                     scan_max_attempts=2, scan_timeout=60000, q=None,
                     **kwargs):
    result = []
    # Initialize versions and names
    DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial,
                                                    blocking=True)()
    try:

        # ########### Preconditions ###############
        # #########################################

        bluetooth_steps.LogInfo("######## SETUP ########")()

        # DUT: turn on BT
        bluetooth_steps.StopPackage(serial=serial, blocking=True)()
        bluetooth_steps.PressHome(serial=serial, blocking=True)()
        bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True,
                                              version=DUT_VERSION,
                                              blocking=True)()
        bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON",
                                             version=DUT_VERSION,
                                             blocking=True)()

        # DUT: wait scanning, rename device and remove all paired devices
        bluetooth_steps.WaitBtScanning(serial=serial,
                                       scan_timeout=scan_timeout,
                                       version=DUT_VERSION, blocking=True)()
        bluetooth_steps.BtRemoveAllPairedDevices(serial=serial,
                                                 version=DUT_VERSION,
                                                 blocking=True)()
        bluetooth_steps.CheckBtVisibility(serial=serial, version=DUT_VERSION,
                                          blocking=True)()

        # ############ Actual Test ################
        # #########################################

        bluetooth_steps.LogInfo("##### ACTUAL TEST #####")()

        bluetooth_steps.PairDevice(serial=serial, dev_to_pair_name=a2dp_device_name,
                                   scan_timeout=scan_timeout, timeout=timeout,
                                   scan_max_attempts=scan_max_attempts)()
        event.set()
        result.append(True)
    except:
        event.set()
        log.error('ConnectA2dpDevice: Unexpected Error')
        log.error(traceback.format_exc())
        result.append(False)
    finally:
        if total_actions > 1:
            event.wait(CONDITION_EVENT_TIMEOUT)
            event.clear()
        # ########### Postconditions ##############
        # #########################################

        bluetooth_steps.LogInfo("####### CLEANUP #######")()

        # DUT: stop settings and turn on BT (if not already)
        bluetooth_steps.StopPackage(serial=serial, critical=False)()
        bluetooth_steps.PressHome(serial=serial, critical=False)()
        bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=True,
                                              version=DUT_VERSION,
                                              critical=False)()
        bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON",
                                             version=DUT_VERSION,
                                             critical=False)()

        # DUT: remove all paired devices and turn off BT
        bluetooth_steps.WaitBtScanning(serial=serial,
                                       scan_timeout=scan_timeout,
                                       version=DUT_VERSION, critical=False)()
        bluetooth_steps.BtRemoveAllPairedDevices(serial=serial,
                                                 version=DUT_VERSION,
                                                 critical=False)()
        bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="OFF",
                                             version=DUT_VERSION,
                                             critical=False)()
        bluetooth_steps.StopPackage(serial=serial, critical=False)()
        bluetooth_steps.PressHome(serial=serial, critical=False)()

    if False in result:
        action_result = ["A2dpBtConnect", 'Fail']
    else:
        action_result = ["A2dpBtDevice", 'Pass']
    return action_result if q is None else q.put(action_result)

def ftp_download_file(serial, condition, event, mode='bg', security='none',
                      ddwrt_ap_name='ddwrt', ddwrt_ap_pass='test1234',
                      encryption=None, file_name='generated.bin',
                      file_size=102400, protocol='ftp', port_number='20211',
                      dut_security=None, conf_security='NONE',
                      compare_method='md5', adb_server_port='5037',
                      iteration=1,
                      q=None):
    result = []
    try:
        platform = statics.Device(serial=serial)

        adb_steps.connect_device(serial=serial, port=adb_server_port)()
        for _ in range(0, iteration):
            if iteration > 1:
                log.info("Ftp download: Iteration #{}".format(_ + 1))
            if priority == 'wifi' and _ == 0:
                # print 'wifi 1st event wait'
                # print e.is_set()
                event.wait(CONDITION_EVENT_TIMEOUT)
                event.clear()
            elif not priority == 'toggle':
                # print 'wifi 1st event set'
                event.set()

            if _ == 0:
                with condition:
                    if priority == 'bt' and _ == 0:
                        condition.wait(CONDITION_EVENT_TIMEOUT)
                    if not connect_to_ap(serial, condition):
                        raise Exception("Not able to connect to AP")

                    # generate URL
                    (URL, IP) = wifi_generic_steps.create_download_url(
                        file_name, file_size, local_path=".", protocol=protocol,
                        port=port_number, serial=serial)()
                    if priority == 'wifi' and _ == 0:
                        # print 'Notify wifi process'
                        condition.notify()

            # print 'Notified after creating url'
            if priority == 'wifi' and _ == 0:
                # print e.is_set()
                # print 'wifi 2nd event wait'
                event.wait(CONDITION_EVENT_TIMEOUT)
            elif not priority == 'toggle':
                # print 'wifi 2nd event set'
                event.set()

            wifi_instance = None
            with condition:
                if priority == 'bt' and _ == 0:
                    condition.wait(CONDITION_EVENT_TIMEOUT)
                # start download
                wifi_instance = wifi_generic_steps.download_file(URL, file_name,
                                                                 file_size,
                                                                 protocol,
                                                                 serial=serial,
                                                                 check_file_downloaded=False)
                wifi_instance()

                if priority == 'wifi' and _ == 0:
                    condition.notify()

                # this condition will notify the 'toggle_bt' to proceed to
                # toggle state
                if priority == 'toggle':
                    event.set()

            # This notify bt operation to start validating receiving file as
            #  this should not happen parallel to wifi download as both
            # rely on UI
            if initiator == "dev" and priority == "bt" and _ == 0:
                event.set()

            wifi_instance.check_file_downloaded = True
            if wifi_instance.check_condition():
                # check download integrity
                if wifi_generic_steps.check_file_integrity(mode=compare_method,
                                                           local_file=file_name,
                                                           serial=serial,
                                                           remote_file=platform.download_path + file_name)():
                    result.append('Pass')
                    log.info("Download file is completed")
                else:
                    raise Exception('File integrity check failed')
            else:
                raise Exception("Error during FTP Download")
            #q.put(['WifiDownloadFile', True])
    except Exception:
        log.error(Exception.message)
        log.error(traceback.format_exc())
        result.append('Fail')
    except:
        log.error('FtpDownloadFile: Unexpected Error')
        log.error(traceback.format_exc())
        result.append('Fail')

    if False in result:
        q.put(['FtpDownloadFile', 'Fail', result])
    else:
        q.put(['FtpDownloadFile', 'Pass'])

    log.info("Wifi Ftp download completed")


def opp_send_file(serial, serial_dev, condition, event,
                  file_path="/storage/emulated/0/acs_files",
                  file_name="10MB.txt", iteration=1, q=None):
    result = []
    try:
        file_full_path = file_path + '/' + file_name
        # Initialize versions and names (we only need dev name)
        DUT_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial,
                                                        blocking=True)()
        DEV_VERSION = bluetooth_steps.GetAndroidVersion(serial=serial_dev,
                                                        blocking=True)()
        PAIRING_DUT_NAME = bluetooth_steps.GetBtMac(serial=serial,
                                                    blocking=True)()
        PAIRING_DEV_NAME = bluetooth_steps.GetBtMac(serial=serial_dev,
                                                    blocking=True)()
        # taking only the name and removing the extensions
        FILE_NAME = file_name.split(".")[0]

        bluetooth_steps.LogInfo("######## SETUP ########")()

        for _ in range(0, iteration):
            if initiator == "dev" and _ == 0:
                serial, serial_dev = serial_dev, serial
                DUT_VERSION, DEV_VERSION = DEV_VERSION, DUT_VERSION
                PAIRING_DUT_NAME, PAIRING_DEV_NAME = PAIRING_DEV_NAME, \
                                                     PAIRING_DUT_NAME

            if iteration > 1:
                log.info("Opp transfer: Iteration #{}".format(_ + 1))
            if priority == 'bt' and _ == 0:
                # print e.is_set()
                # print 'bt 1st event wait'
                event.wait(CONDITION_EVENT_TIMEOUT)
                event.clear()
            elif not priority == "toggle":
                # print 'bt 1st event set'
                event.set()

            with condition:
                # print 'Before wifi'
                if priority == 'wifi' and _ == 0:
                    condition.wait(CONDITION_EVENT_TIMEOUT)
                # print 'Started after notified from wifi'
                # DUT: clear all notifications etc
                bluetooth_steps.StopPackage(serial=serial, blocking=True)()
                bluetooth_steps.PressHome(serial=serial, blocking=True)()
                bluetooth_steps.ClearAllNotifications(serial=serial,
                                                      version=DUT_VERSION,
                                                      blocking=True)()
                bluetooth_steps.BtOppDismissEventualIncoming(serial=serial,
                                                             version=DUT_VERSION,
                                                             blocking=True)()
                bluetooth_steps.ClearRecentApps(serial=serial,
                                                    version=DUT_VERSION,
                                                    blocking=True)()

                # DEV: clear all notifications/opp files etc
                bluetooth_steps.StopPackage(serial=serial_dev, blocking=True)()
                bluetooth_steps.PressHome(serial=serial_dev, blocking=True)()
                bluetooth_steps.ClearAllNotifications(serial=serial_dev,
                                                      version=DEV_VERSION,
                                                      blocking=True)()
                bluetooth_steps.BtOppDismissEventualIncoming(serial=serial_dev,
                                                             version=DEV_VERSION,
                                                             blocking=True)()
                bluetooth_steps.ClearPath(serial=serial_dev, blocking=True)()

                if _ == 0:
                    # DUT: turn on bt
                    # already turned on during TC setup phase
                    bluetooth_steps.OpenBluetoothSettings(serial=serial,
                                                          use_intent=True,
                                                          version=DUT_VERSION,
                                                          blocking=True)()
                    bluetooth_steps.ClickBluetoothSwitch(serial=serial, state="ON",
                                                         version=DUT_VERSION,
                                                         blocking=True)()

                    # DEV: turn on bt
                    # already turned on during TC setup phase
                    bluetooth_steps.OpenBluetoothSettings(serial=serial_dev,
                                                          use_intent=True,
                                                          version=DEV_VERSION,
                                                          blocking=True)()
                    bluetooth_steps.ClickBluetoothSwitch(serial=serial_dev,
                                                         state="ON",
                                                         version=DEV_VERSION,
                                                         blocking=True)()

                    # DUT: wait scan and remove paired devices
                    bluetooth_steps.WaitBtScanning(serial=serial,
                                                   version=DUT_VERSION,
                                                   blocking=True)()
                    bluetooth_steps.BtChangeDeviceName(serial=serial,
                                                       name=PAIRING_DUT_NAME,
                                                       version=DUT_VERSION,
                                                       blocking=True)()
                    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial,
                                                             version=DUT_VERSION,
                                                             blocking=True)()
                    bluetooth_steps.CheckBtVisibility(serial=serial,
                                                      version=DUT_VERSION,
                                                      blocking=True)()

                    # DEV:  wait scan (should be already finished), rename and
                    # remove paired devices
                    bluetooth_steps.WaitBtScanning(serial=serial_dev,
                                                   timeout_appear=0,
                                                   version=DEV_VERSION,
                                                   blocking=True)()
                    bluetooth_steps.BtChangeDeviceName(serial=serial_dev,
                                                       name=PAIRING_DEV_NAME,
                                                       version=DEV_VERSION,
                                                       blocking=True)()
                    bluetooth_steps.BtRemoveAllPairedDevices(serial=serial_dev,
                                                             version=DEV_VERSION,
                                                             blocking=True)()
                    bluetooth_steps.CheckBtVisibility(serial=serial_dev,
                                                      version=DEV_VERSION,
                                                      blocking=True)()
                    if priority == 'bt' and _ == 0:
                        condition.notify()

            if _ == 0 and bt_pair_device:
                if not pair_device(serial, serial_dev, PAIRING_DUT_NAME,
                        PAIRING_DEV_NAME, DUT_VERSION, DEV_VERSION):
                    raise

            if priority == 'bt' and _ == 0:
                # print e.is_set()
                # print 'bt 2nd event wait'
                event.wait(CONDITION_EVENT_TIMEOUT)
                event.clear()
            elif not priority == 'toggle':
                # print 'bt 2nd event set'
                event.set()

            with condition:
                if priority == 'wifi' and _ == 0:
                    condition.wait(CONDITION_EVENT_TIMEOUT)

                bt_opp_receive_obj = None
                bluetooth_steps.BrowseFileInSettingsStorage(serial=serial,
                                                            file_full_path=file_full_path)()
                bluetooth_steps.BtOppShareFile(serial=serial,
                                               server_dut=str(PAIRING_DEV_NAME),
                                               bt_already_opened=True,
                                               version=DUT_VERSION,
                                               blocking=True)()
                bt_opp_receive_obj = bluetooth_steps.BtOppReceiveFile(
                    serial=serial_dev,
                    action='Accept',
                    filename_starting_string=FILE_NAME,
                    version=DEV_VERSION,
                    check_file_received=False)
                bt_opp_receive_obj()
                if priority == 'bt' and _ == 0:
                    condition.notify()

                # this condition will notify the 'toggle_wifi' to proceed to
                # toggle state
                if priority == 'toggle':
                    event.set()

            # This waits for Wifi to start downloas as both collides when
            # start together
            if initiator == "dev" and priority == "bt" and _ == 0:
                event.wait(CONDITION_EVENT_TIMEOUT)
                event.clear()

            # Below verification of OPP received file is separated from
            # above BtOppReceiveFile because parallelly when file is receiving
            # toggling WIFI should happen
            try:
                # open notification menu and click on incoming file notification
                if not bluetooth_steps.OpenNotificationsMenu(serial=serial_dev,
                                                             version=DEV_VERSION,
                                                             critical=False,
                                                             no_log=True)():
                    raise Exception("Notifications menu not opened")
                if not bluetooth_steps.BtCheckNotificationAppear(serial=serial_dev,
                                                                 text_contains="Bluetooth share: Receiving",
                                                                 click_on_notification=True,
                                                                 time_to_appear=VIEW_TIMEOUT,
                                                                 version=DEV_VERSION,
                                                                 critical=False,
                                                                 no_log=True)():
                    if not bluetooth_steps.BtCheckNotificationAppear(serial=serial_dev,
                                                                 text_contains="Bluetooth share: Received",
                                                                 click_on_notification=True,
                                                                 time_to_appear=VIEW_TIMEOUT,
                                                                 version=DEV_VERSION,
                                                                 critical=False,
                                                                 no_log=True)():
                        raise Exception("Receiving file notification click failed")
                    else:
                        if not bt_opp_receive_obj.uidevice(
                                textContains=FILE_NAME).wait.exists(timeout=VIEW_TIMEOUT):
                            raise Exception("Receiving file not found in "
                                            "Inbound transfers window")
                        if not bt_opp_receive_obj.uidevice(
                                    textContains="Received complete"):
                            raise Exception("File not received completely")
                        bt_opp_receive_obj.uidevice.press.back()
                else:
                    if not bt_opp_receive_obj.uidevice(
                            resourceId="android:id/alertTitle",
                            text="File transfer").wait.exists(timeout=VIEW_TIMEOUT):
                        raise Exception("File transfer progress window not "
                                        "shown after click on receiving file notification")
                    # check the filename in the window (if not empty)
                    if FILE_NAME != "":
                        filename_obj = bt_opp_receive_obj.uidevice(
                            textContains="File:")
                        if not filename_obj.wait.exists(timeout=VIEW_TIMEOUT):
                            raise Exception(
                                "Receiving File Name not found in the file "
                                "transfer window")
                        filename_text = str(filename_obj.text)
                        if FILE_NAME not in filename_text:
                            raise Exception("Not expected string contained in the "
                                            "file transfer window, exp " + str(
                                FILE_NAME) + " but found " + filename_text)
                    # wait till the file is received
                    if not bt_opp_receive_obj.uidevice(
                            text="File received").wait.exists(
                            timeout=TRANSFER_TIMEOUT):
                        if bt_opp_receive_obj.uidevice(textContains="OK",
                                        className="android.widget.Button").exists:
                            bt_opp_receive_obj.uidevice.press.back()
                            #bt_opp_receive_obj.uidevice(textContains="OK",
                            #
                            # className="android.widget.Button").click.wait()
                        raise Exception("Timeout reached, file transfer not finished")
                    # return to the previous window
                    bt_opp_receive_obj.uidevice.press.back()
                    if not bt_opp_receive_obj.uidevice(
                            resourceId="android:id/alertTitle",
                            text="File transfer").wait.gone(timeout=VIEW_TIMEOUT):
                        #if bt_opp_receive_obj.uidevice(textContains="OK",
                        #            className="android.widget.Button").exists:
                        #    bt_opp_receive_obj.uidevice.press.back()
                            #bt_opp_receive_obj.uidevice(textContains="OK",
                            #           className="android.widget.Button").click()

                        # this will help avoid the "File transfer" window
                        # appear in next opp transfer
                        bt_opp_receive_obj.uidevice.press.back()
                        raise Exception("File transfer progress window not closed")
                log.info("File received successfully")
                result.append(True)
            except Exception, e:
                log.error(e.message)
                log.error(traceback.format_exc())
                log.error("Exception raised, File not received successfully")
                result.append(False)
    except:
        log.error('OppTransferFile: Unexpected Error')
        log.error(traceback.format_exc())
        result.append(False)
    finally:
        bluetooth_steps.LogInfo("####### CLEANUP #######")()

        # DUT: close inbounds transfers, turn off bt, clear all opp notifications/files
        bluetooth_steps.DismissTransferList(serial=serial, version=DUT_VERSION,
                                            critical=False)()
        bluetooth_steps.StopPackage(serial=serial, critical=False)()
        bluetooth_steps.ClearAllNotifications(serial=serial,
                                              version=DUT_VERSION,
                                              critical=False)()
        bluetooth_steps.PressHome(serial=serial, critical=False)()

        # DEV: turn off bt, clear all opp notifications/files
        bluetooth_steps.ClearAllNotifications(serial=serial_dev,
                                              version=DEV_VERSION,
                                              critical=False)()
        bluetooth_steps.StopPackage(serial=serial_dev, critical=False)()
        bluetooth_steps.PressHome(serial=serial_dev, critical=False)()

    if False in result:
        q.put(['OppTransferFile', 'Fail', result])
    else:
        q.put(['OppTransferFile', 'Pass'])
        # Teardown
    log.info("BT Opp transfer completed")


def compute_result(q):
    result = 'Pass'
    results = []
    action_failed = []

    while not q.empty():
        action_and_result = q.get()
        results.append(action_and_result)
        if "Fail" in action_and_result:
            action_failed.append(action_and_result[0])
            result = "Fail"
    return result, action_failed, results


##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
serial2 = args["serial2"]
# possible values of action are
# BtOnOff:1,WifiOnOff:2,FtpDownloadFile:3,BtOppTransferFile:2
action = args["action"]

# all the actions are assigned to "ordered dictonary because action will be
# performed in the same order as mentioned except "WifiOnOff and BtOnOff" as
#  these two will be performed during transfer progress
actions = OrderedDict()
for action, iterat in [item.split(":") for item in action.split(",")]:
    actions[action] = int(iterat)

# optional params for 'FtpDownloadFile' action
file_size = args.get("file_size", None)

# below paramaters can also be added if further customization is needed for
# AP connection and others
# security = args["security"]
# dut_security = args["dut_security"]
# ddwrt_ap_name = args["ap_name"]

# optional params for 'OppTransferFile' action
# valid value for initiator are ("dut", "dev") and defaults to "dut"
initiator = args.get("bt_initiator", "dut")
bt_pair_device = args.get("bt_pair", False)

# optional params for 'A2dpBtConnect' action
bt_a2dp_name = args.get("bt_name", None)

##### test start #####

result = []

wifi_bt_mutual_exclusive_condition = Condition()
wifi_bt_sync_event = Event()
result_q = Queue()
message_q = Queue()

# checking mutually exclusive actions
if "BtOnOff" in actions and "OppTransferFile" in actions:
    log.error("Mutually exclusive actions, BtOnOff and OppTransferFile cannot "
              "happen together")
    exit(1)

if "WifiOnOff" in actions and "FtpDownloadFile" in actions:
    log.error("Mutually exclusive actions, WifiOnOff and FtpDownloadFile "
              "cannot happen together")
    exit(1)

if "BtOnOff" in actions and "A2dpBtConnect" in actions:
    log.error("Mutually exclusive actions, BtOnOff and A2dpBtConnect cannot "
              "happen together")
    exit(1)

if "A2dpBtConnect" in actions and ("OppTransferFile" in actions or
                                           "FtpDownloadFile" in actions):
    log.error("Mutually exclusive actions, A2dpBtConnect and OppTransferFile or "
              "FtpDownloadFile cannot happen together")
    exit(1)
# Check for more than one action's iteration is more than 1
multiple_iteration_count = 0
for a in valid_actions:
    try:
        if actions[a] > 1 :
            multiple_iteration_count += 1
        if multiple_iteration_count > 1:
            log.error("Invalid iteration count as only one operation can iterate "
                      "multiple times")
            exit(1)
    except:
        pass

if "BtOnOff" in actions:
    check_power_for = "wifi"
elif "WifiOnOff" in actions:
    check_power_for = "bt"
else:
    check_power_for = "wifi"

main_process = None
process = []
total_actions = 0
log.debug("action key is {}".format(actions))

# for action in actions.keys():
if "FtpDownloadFile" in actions.keys():
    total_actions += 1
    iteration_count = actions['FtpDownloadFile']
    # print 'wifi', iteration
    if iteration_count == 1:
        # 100MB data
        f_size = 102400 if file_size is None else file_size
        try:
            if "OppTransferFile" in actions.keys():
                if actions.keys().index("FtpDownloadFile") < actions.keys(
                        ).index("OppTransferFile") or actions["OppTransferFile"] > 1:
                    priority = 'wifi'
            else:
                priority = "wifi"
        except:
            # print 'In wifi exception'
            pass
    elif 1 < iteration_count <= 5:
        # 50MB data
        f_size = 51200 if file_size is None else file_size
    else:
        f_size = 10240 if file_size is None else file_size

    if main_process is None:
        p1 = Process(name=check_power_for + ' power status check periodically',
                     target=check_power_status_periodically,
                     args=(serial, check_power_for, message_q),
                     kwargs={'q': result_q})
        p1.start()
        main_process = p1

    p2 = Process(name="FtpDownloadFile", target=ftp_download_file,
                 args=(serial, wifi_bt_mutual_exclusive_condition,
                       wifi_bt_sync_event),
                 kwargs={'file_size': f_size, 'iteration': iteration_count,
                         'q': result_q})
    # p2.start()
    process.append(p2)

if "OppTransferFile" in actions.keys():
    total_actions += 1
    iteration_count = actions["OppTransferFile"]
    # print 'bt', iteration
    if iteration_count == 1:
        f_name = '10MB.txt'
        # if priority == None:
        #    priority = 'bt'
        try:
            if "FtpDownloadFile" in actions.keys():
                if actions.keys().index("OppTransferFile") < actions.keys().index(
                        "FtpDownloadFile") or actions["FtpDownloadFile"] > 1:
                    priority = 'bt'
            else:
                priority = 'bt'
        except:
            # print 'in bt exception'
            pass
    elif 1 < iteration_count <= 5:
        f_name = '5MB.txt'
    else:
        f_name = '1MB.txt'

    if main_process is None:
        p1 = Process(name=check_power_for + ' power status check periodically',
                     target=check_power_status_periodically,
                     args=(serial, check_power_for, message_q),
                     kwargs={'q': result_q})
        p1.start()
        main_process = p1

    p2 = Process(name="OppTransferFile", target=opp_send_file,
                 args=(serial, serial2,
                       wifi_bt_mutual_exclusive_condition, wifi_bt_sync_event),
                 kwargs={'file_name': f_name, 'iteration': iteration_count,
                         'q': result_q})
    # p2.start()
    process.append(p2)

if 'BtOnOff' in actions.keys():
    total_actions += 1

    if priority is None:
        wifi_bt_sync_event.set()
    else:
        priority = 'toggle'
    p1 = Process(name="BtOnOff", target=toggle_bt,
                 args=(serial, wifi_bt_sync_event),
                 kwargs={'iteration': actions['BtOnOff'], 'q': result_q})
    # p1.start()
    process.append(p1)

if 'WifiOnOff' in actions.keys():
    total_actions += 1
    if priority is None:
        wifi_bt_sync_event.set()
    else:
        priority = 'toggle'
    p1 = Process(name="WifiOnOff", target=toggle_wifi,
                 args=(serial, wifi_bt_sync_event),
                 kwargs={'iteration': actions["WifiOnOff"], 'q': result_q})
    # p1.start()
    process.append(p1)

if "A2dpBtConnect" in actions.keys():
    total_actions += 1
    priority = "a2dp"
    wifi_bt_sync_event.clear()
    p1 = Process(name="A2dpConnect", target=pair_a2dp_device,
                 args=(serial, bt_a2dp_name, wifi_bt_sync_event),
                 kwargs={'q': result_q})
    process.append(p1)

# if 'ConnectToAp' in actions.keys():
#    total_actions += 1
#    p1 = Process(name="ConnectAp", target=connectToAp, args=(serial, ),
#                 kwargs={'iteration': actions["ConnectToAp"], 'q': result_q})
#    #p1.start()
#    process.append(p1)

if multiple_iteration_count > 1:
    log.error("Invalid iteration count as only one operation can iterate "
              "multiple times")
else:
    if total_actions == 1:
        priority = None

    #print 'priority is', priority
    #print 'All processes are ', process
    for p in process:
        p.start()

#    print active_children()
    for p in process:
        p.join(JOIN_TIMEOUT)

#print 'Atleast one process is active'
message_q.put('terminate')

if main_process:
    main_process.join()

try:
    result, action_failed, all_result = compute_result(result_q)
    log.debug(all_result)
    if result == "Fail":
        raise Exception('TEST FAILED during {}'.format(action_failed))
except Exception as e:
    raise FailedError(e.message)
else:
    log.info('TEST PASSED')


##### test end #####
