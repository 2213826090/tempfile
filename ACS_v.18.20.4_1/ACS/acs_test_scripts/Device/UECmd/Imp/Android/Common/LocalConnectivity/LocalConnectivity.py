"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This file implements the Localconnectivity UEcmd for Android phone
:since: 12/04/2011
:author: vgombert
"""

import os
import re
import posixpath
import struct
import tempfile
import time
from datetime import datetime
from UtilitiesFWK.Utilities import get_method_name, FINDKEY, Global
from acs_test_scripts.Device.UECmd.UECmdTypes import BluetoothDevice, BtServiceClass
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Misc.PhoneSystem import PhoneSystem
from acs_test_scripts.Device.UECmd.Interface.LocalConnectivity.ILocalConnectivity import ILocalConnectivity
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE, BT_SCAN_MODE, BT_BOND_STATE, \
    BT_PINVARIANT, BtConState, BtAudioState, BtProfile
from Device.Model.AndroidDevice.CrashInfo.CrashInfo import CrashInfo
from acs_test_scripts.Device.UECmd.UECmdDecorator import need


import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
import acs_test_scripts.Utilities.LocalConnectivityUtilities as LocalConnectivityUtil
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from uiautomator import Device

class BondState(object):

    """
    Class enumerating Bond state value :
    cf. http://developer.android.com/reference/android/bluetooth/BluetoothDevice.html
    """
    NAME = "Bond_state"
    BOND_NONE = 10
    BOND_BONDING = 11
    BOND_BONDED = 12


class ScanMode(object):

    """
    Class enumerating Scan mode value :
    cf. http://developer.android.com/reference/android/bluetooth/BluetoothAdapter.html
    """
    NAME = "mode"
    SCAN_MODE_NONE = 20
    SCAN_MODE_CONNECTABLE = 21
    SCAN_MODE_CONNECTABLE_DISCOVERABLE = 23


class BtState(object):

    """
    Class enumerating Bluetooth state value :
    cf. http://developer.android.com/reference/android/bluetooth/BluetoothAdapter.html
    """
    NAME = "Power_state"
    STATE_CONNECTED = 2
    STATE_CONNECTING = 1
    STATE_DISCONNECTED = 0
    STATE_DISCONNECTING = 3
    STATE_OFF = 10
    STATE_ON = 12
    STATE_TURNING_OFF = 13
    STATE_TURNING_ON = 11


class BtPinVariant(object):

    """
    Class enumerating Bluetooth pin variant for pairing process :
    """
    NAME = "PinVariant"
    # No Pin variant set
    ERROR = -1
    # The user will be prompted to enter a pin
    PIN = 0
    # The user will be prompted to enter a passkey
    PASSKEY = 1
    # The user will be prompted to confirm the passkey displayed on the screen
    PASSKEY_CONFIRMATION = 2
    # The user will be prompted to accept or deny the incoming pairing request
    CONSENT = 3
    # The user will be prompted to enter the passkey displayed on remote device
    # This is used for Bluetooth 2.1 pairing.
    DISPLAY_PASSKEY = 4
    # The user will be prompted to enter the PIN displayed on remote device.
    # This is used for Bluetooth 2.0 pairing.
    DISPLAY_PIN = 5
    # The user will be prompted to accept or deny the OOB pairing request
    OOB_CONSENT = 6


class NfcP2pConfiguration(object):

    """
    Class enumerating NFC P2P configurations :
    """
    P2P_INITIATOR_CONFIGURATION = {"DISABLED": 0x00,
                                   "DEFAULT": 0x27,
                                   "PASSIVE_106": 0x01,
                                   "PASSIVE_212": 0x02,
                                   "PASSIVE_424": 0x04,
                                   "ACTIVE_106": 0x08,
                                   "ACTIVE_212": 0x10,
                                   "ACTIVE_424": 0x20}

    P2P_TARGET_CONFIGURATION = {"DISABLED": 0x00,
                                "DEFAULT": 0x0E,
                                "PASSIVE_ALL": 0x07,
                                "ACTIVE_ALL": 0x08}


class LocalConnectivity(BaseV2, ILocalConnectivity):

    """
    :summary: Local Connectivity UEcommands operations for Android platform
    that handle all BT operations
    """

    _BLUETOOTH_MODULE = "acscmd.connectivity.bt.BluetoothModule"
    _NFC_MODULE = "acscmd.connectivity.nfc.NfcModule"
    _WAIT_FOR_POLL_SECS = 1  # Time to wait before polling again

    def __init__(self, device):
        """
        Constructor
        """

        BaseV2.__init__(self, device)
        ILocalConnectivity.__init__(self, device)
        self._logger = device.get_logger()
        self.component = "com.intel.acs.agent/.LocalConnectivity"
        self._phone_system = PhoneSystem(device)
        self._last_tag_op_code = ""
        self._device_serial_number = device.retrieve_serial_number()
        self._bt_iface = device.get_config("btIface", "hci0")

    @need('bluetooth')
    def get_bt_scan_mode(self):
        """
        Retrieves the Bluetooth scan mode.

        :rtype: str
        :return: scan mode (both, noscan, inquiry)
        """
        method = "getScanMode"

        mode = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)["result"]
        return mode

    @need('bluetooth')
    def bt_service_browsing(self, bd_address, class_to_browse=""):
        """
        Remote device service browsing.

        :type bd_address: str
        :param bd_address: Remote device BD address
                           format 00:00:00:00:00:00

        :type class_to_browse: str
        :param class_to_browse: type of service class

        :rtype: Boolean
        :return: Service search result
         True  = Service found or browsing succeeded
         False = Service not found or browsing failed
        :rtype: BluetoothDevice
        :return: an instance of BluetoothDevice with
         Name of remote device
         Address of remote device
         uuids = list of Services found
        """

        # Check address format
        self._validate_bt_address(bd_address)

        # Check class_to_browse is valid
        if class_to_browse != "":
            # Find a specific class in the result list
            if class_to_browse in BtServiceClass.BT_SERVICE_CLASS:
                self._logger.info("Searching class = %s <%s>"
                                  % (class_to_browse, BtServiceClass.BT_SERVICE_CLASS[class_to_browse]))
            else:
                msg = "bt_service_browsing : Parameter class_to_browse %s is not valid" % class_to_browse
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        method = "getServiceRecord"
        args = "--es Address %s" % bd_address

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        btdev = BluetoothDevice()
        for element in output:
            if "Address" in element:
                btdev.address = element["Address"]
            if "Name" in element:
                btdev.name = element["Name"]
            if "Uuid" in element:
                uuid = element["Uuid"].upper()
                btdev.uuids.append(uuid)
                self._logger.info("service found: %s"
                                  % FINDKEY(BtServiceClass.BT_SERVICE_CLASS, uuid))

        if class_to_browse != "":
            # Find a specific class in the result list
            if BtServiceClass.BT_SERVICE_CLASS[class_to_browse] in btdev.uuids:
                return True, btdev
            else:
                # Service not found in the list
                return False, btdev
        else:
            # Retrieve all the service supported by remote
            if len(btdev.uuids) > 0:
                # at least one service has been found
                return True, btdev
            else:
                # No service found
                return False, btdev

    @need('bluetooth')
    def bt_l2cap_ping(self, bd_address, packet_count="", packet_size=""):
        """
        Performs a l2ping on a remote Bluetooth device.

        :type bd_address: str
        :param bd_address: Bluetooth address to ping

        :type packet_size: int
        :param packet_size: Packet size in bytes

        :type packet_count: int
        :param packet_count: Number of packet to send

        :rtype: Measure Object (value,unit)
        :return: packet loss
        """
        output = "%s not implemented on Android" % get_method_name()
        self._logger.warning(output)
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, output)

    @need('bluetooth')
    def bt_reset_device(self):
        """
        Resets a Bluetooth adapter. Turn OFF then ON and check power state is successfully ON

        :return: None
        """
        method = "resetPower"
        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)

    @need('bluetooth')
    def bt_scan_devices(self):
        """
        Scans remote Bluetooth devices.

        :rtype: list of BluetoothDevice object
        :return: list of founded adapters (name/address)
        """
        method = "scan"
        device_list = []

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_MODULE, method, is_system=True)

        for element in output:
            btdev = BluetoothDevice()
            if "Address" in element:
                btdev.address = element["Address"]
            if "Name" in element:
                btdev.name = element["Name"]
            device_list.append(btdev)

        return device_list

    @need('bluetooth')
    def bt_scan_devices_interleave_search(self):
        """
        Scans remote Bluetooth devices.

        :rtype: list of BluetoothDevice object
        :return: list of founded adapters (name/address)
        """
        method = "scanInterleave"
        device_list = []

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_MODULE, method, is_system=True)

        for element in output:
            btdev = BluetoothDevice()
            if "Address" in element:
                btdev.address = element["Address"]
            if "Name" in element:
                btdev.name = element["Name"]
            device_list.append(btdev)

        return device_list

    @need('bluetooth')
    def bt_find_device(self, remote_device_info):
        """
        Looks for a Bluetooth device.

        :type remote_device_info: str
        :param remote_device_info: Name or address of the device

        :rtype: boolean
        :return: True if found , False otherwise
        """
        bt_found = False
        bt_devices_list = self.bt_scan_devices()

        if NetworkingUtil.is_valid_mac_address(remote_device_info):
            # remote_device_info is a bd address
            for device in bt_devices_list:
                if (str(device.address).upper() == str(remote_device_info).upper() or
                    str(device.name).upper() == str(remote_device_info).upper()):
                    bt_found = True
                    self._logger.info("Device %s found" % remote_device_info)
                    break
        else:
            # remote_device_info is not a valid bd address, maybe a name !
            for device in bt_devices_list:
                if str(device.name) == str(remote_device_info):
                    bt_found = True
                    self._logger.info("Device %s found" % remote_device_info)
                    break

        if not bt_found:
            self._logger.info("Device " + remote_device_info +
                              " is not in range !")
        return bt_found

    @need('bluetooth')
    def bt_find_device_interleave_search(self, remote_device_info):
        """
        Looks for a Bluetooth device.

        :type remote_device_info: str
        :param remote_device_info: Name or address of the device

        :rtype: boolean
        :return: True if found , False otherwise
        """
        bt_found = False
        bt_devices_list = self.bt_scan_devices_interleave_search()

        if NetworkingUtil.is_valid_mac_address(remote_device_info):
            # remote_device_info is a bd address
            for device in bt_devices_list:
                if (str(device.address).upper() == str(remote_device_info).upper() or
                    str(device.name).upper() == str(remote_device_info).upper()):
                    bt_found = True
                    self._logger.info("Device %s found" % remote_device_info)
                    break
        else:
            # remote_device_info is not a valid bd address, maybe a name !
            for device in bt_devices_list:
                if str(device.name) == str(remote_device_info):
                    bt_found = True
                    self._logger.info("Device %s found" % remote_device_info)
                    break

        if not bt_found:
            self._logger.info("Device " + remote_device_info +
                              " is not in range !")
        return bt_found

    @need('bluetooth')
    def bt_find_multi_devices(self, remote_devices_info, tries):
        """
        Scans and finds for multiple Bluetooth device.

        :type remote_devices_info: list of str
        :param remote_devices_info: Names or addresses of the devices
        :type tries: int
        :param tries: The maximum number of tries (including a new scan) you want to give to find the devices.

        :rtype: boolean
        :return: True if found , False otherwise
        """

        all_devices_found = False
        tries = int(tries)
        for cur_try in range(int(tries)):
            bt_found = 0
            self._logger.info("bt_find_multi_device - try %d/%d" % ((cur_try + 1), tries))
            bt_devices_list = self.bt_scan_devices()
            for remote_device_info in remote_devices_info:
                cur_dev_found = False
                if NetworkingUtil.is_valid_mac_address(remote_device_info):
                    # remote_device_info is a bd address
                    for device in bt_devices_list:
                        if (str(device.address).upper() == str(remote_device_info).upper() or
                            str(device.name).upper() == str(remote_device_info).upper()):
                            bt_found = bt_found + 1
                            cur_dev_found = True
                            self._logger.info("Device %s found" % remote_device_info)
                            break
                else:
                    # remote_device_info is not a valid bd address, maybe a name !
                    for device in bt_devices_list:
                        if str(device.name) == str(remote_device_info):
                            bt_found = bt_found + 1
                            cur_dev_found = True
                            self._logger.info("Device %s found" % remote_device_info)
                            break
                all_devices_found = (bt_found == len(remote_devices_info))
                if cur_dev_found == False:
                    self._logger.info("Device " + remote_device_info +
                                      " is not in range !")
            if all_devices_found:
                break
        return all_devices_found

    @need('bluetooth')
    def bt_check_msg(self, msg):
        """
        check if /sdcard/Bluetooth_Received_File contain this message -- msg
        """
        method = "checkMessage"
        msglength = len(msg)
        cmd = "--es Message '%s' --ei msglength %d" % (msg, msglength)
        self._internal_exec_v2(LocalConnectivity._BLUETOOTH_MODULE, method, cmd, is_system=True)

    @need('bluetooth')
    def bt_receive_msg(self):
        """
        Receive message from another devices through BluetoothSocket
        :return: None
        """
        method = "waitForMessage"
        self._internal_exec_v2(LocalConnectivity._BLUETOOTH_MODULE, method, is_system=True)

    @need('bluetooth')
    def bt_send_msg(self, remote_device_addr, msg):
        """
        Send message to another devices through BluetoothSocket
        :type remote_device_addr: str
        :param remote_device_addr: address of the device
        :type msg: str
        :param msg: message to send
        :return: None
        """
        method = "sendMessage"
        cmd = "--es Address %s --es Message '%s'" % (remote_device_addr, msg)
        self._internal_exec_v2(LocalConnectivity._BLUETOOTH_MODULE, method, cmd, is_system=True)

    @need('bluetooth')
    def bt_opp_init(self, file_to_be_received):
        """
        If already in filesystem, remove the file to be received.
        :type file_to_be_received: str
        :param file_to_be_received: filename of the file to be received soon
        :return: None
        """
        file2delete = "/sdcard/bluetooth/%s" % file_to_be_received
        self._phone_system.delete(file2delete, False)

    @need('bluetooth')
    def bt_opp_clean_notification_list(self):
        """
        Clean (Empty) the BT OPP notification list
        """
        self._logger.info("BT Clean Opp service list")
        method = "emptyOPPNotificationsList"

        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)

    @need('bluetooth')
    def bt_opp_check_service_raw(self):
        """
        Retreive the entire BT OPP notifications information.

        :rtype: dict
        :return: a dictionary containing the following information
        - id: The notification ID.
        - address: The address of remote device concerned by the transfer.
        - filename: The concerned file name.
        - filesize: The concerned file size.
        - downloadedsize: The file size already downloaded.
        - status: The notification status.
        - timeStamp: The timestamp (start time) of the file transfer, in ms.
        - curtime: The current time of the transfer, in ms.
        - direction: the direction of the transfer (UL=0, DL=1)
        """

        method = "checkOPPService"

        raw_ssids = self._internal_exec_multiple_v2(self._BLUETOOTH_MODULE, method, is_system=True)

        return raw_ssids

    @need('bluetooth')
    def bt_opp_check_service(self):
        """
        Retreive the entire BT OPP notifications information.

        :rtype: tuple
        :return: a tuple of notification list with following data:
        - id: The notification ID.
        - address: The address of remote device concerned by the transfer.
        - filename: The concerned file name.
        - filesize: The concerned file size.
        - downloadedsize: The file size already downloaded.
        - status: The notification status.
        - timeStamp: The timestamp (start time) of the file transfer, in ms.
        - curtime: The current time of the transfer, in ms.
        - direction: the direction of the transfer (UL=0, DL=1)
        """

        raw_ssids = self.bt_opp_check_service_raw()

        iddata = self._build_list_from_dict(raw_ssids, "id")
        address = self._build_list_from_dict(raw_ssids, "address")
        filename = self._build_list_from_dict(raw_ssids, "filename")
        filesize = self._build_list_from_dict(raw_ssids, "filesize")
        downloadedsize = self._build_list_from_dict(raw_ssids, "downloadedsize")
        status = self._build_list_from_dict(raw_ssids, "status")
        timestamp = self._build_list_from_dict(raw_ssids, "timestamp")
        curtime = self._build_list_from_dict(raw_ssids, "currenttime")
        direction = self._build_list_from_dict(raw_ssids, "direction")

        return iddata, address, filename, filesize, downloadedsize, status, timestamp, curtime, direction

    @need('bluetooth')
    def bt_opp_send_file(self, filename, destination_address):
        """
        Send a file to another devices through Bluetooth OPP transfer
        WARNING: Display shall be ON before using such method
                 Use phone_system.display_on() + phone_system.set_phone_lock(0)
        :type filename: str
        :param filename: full path and filename of the file to transfer
        :type destination_address: str
        :param destination_address: BT address of the device to send the file to
        :return: None
        """
        KEYCODE_HOME = "3"
        KEYCODE_BACK = "4"

        self._logger.info("Send the file")
        method = "sendOPPfile"

        version = self._exec("adb shell getprop ro.build.version.release")
        # N version manual execute Send the file
        if version.startswith('7'):
            folder=filename.split('/')[2]
            filenames=filename.split(',')
            files=[]
            for file in filenames:
                filename=file.split('/')[3]
                files.append(filename)
            self._exec("adb shell am start -n com.android.settings/com.android.settings.Settings")
            dut=Device(self._device_serial_number)
            settings_list = dut(resourceId="com.android.settings:id/dashboard_container")
            if not settings_list.wait.exists(timeout=30):
                raise Exception("Settings list was not found")
            if not settings_list.scroll.to(text="Storage"):
                raise Exception("Storage option not found")
            dut(text="Storage").click()
            if not dut(text="Internal shared storage").wait.exists(timeout=30):
                raise Exception("Internal shared storage not found")
            dut(text="Internal shared storage").click()
            if not dut(text="Other").wait.exists(timeout=30):
                raise Exception("Other not found")
            dut(text="Other").click()
            if not dut(text="EXPLORE").wait.exists(timeout=30):
                raise Exception("EXPLORE not found")
            dut(text="EXPLORE").click()
            if dut(className="android.widget.ListView").child(className="android.widget.LinearLayout").wait.exists(timeout=30):
                dut(text=folder).click.wait()
            else:
                if dut(text="Download").wait.exists(timeout=30):
                    if not dut(text=folder).wait.exists(timeout=30):
                        raise Exception("%s not found" % folder)
                    dut(text=folder).click.wait()
            for file in files:
                file_list = dut(resourceId="com.android.documentsui:id/dir_list")
                if not file_list.scroll.to(text=file):
                    raise Exception("%s not found" % file)
                dut(text=file).swipe.up(steps=100)
            if dut(resourceId="com.android.documentsui:id/menu_list").wait.exists(timeout=30):
                dut(resourceId="com.android.documentsui:id/menu_list").click()
            elif dut(resourceId="com.android.documentsui:id/menu_grid").wait.exists(timeout=30):
                dut(resourceId="com.android.documentsui:id/menu_grid").click()
            if dut(text="Bluetooth").wait.exists(timeout=30):
                dut(text="Bluetooth").click.wait()
                time.sleep(10)
                if not dut(text=destination_address).wait.exists(timeout=60):
                    raise Exception("Bluetooth share %s not found" % destination_address)
                dut(text=destination_address).click.wait()
        else:
            # M version ACS tool execute Send the file
            args = "--es Address %s --es fileName %s" % (destination_address, filename)
            self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, timeout=120, is_system=True)

        # Send a Back key event to leave the BT devices selector dialog.
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        # Send a Home key event to be sure to return to phone / tablette home.
        self._exec("adb shell input keyevent " + KEYCODE_HOME)

    @need('bluetooth')
    def bt_opp_cancel_send(self):
        """
        Cancel sending files through OPP
        All the files being sent are cancelled
        :return: None
        """
        keycode_home = "3"

        self._logger.info("Cancel sending files")
        method = "cancelOPPTransfer"

        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)

        # Send a Home key event to be sure to return to phone / tablette home.
        self._exec("adb shell input keyevent %s" % keycode_home)

    @need('bluetooth')
    def bt_opp_reject_file(self):
        """
        Reject receiving files through OPP
        All the files being sent are cancelled
        :return: None
        """
        keycode_home = "3"

        self._logger.info("Reject receiving files")
        method = "rejectOPPTransfer"

        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)

        # Send a Home key event to be sure to return to phone / tablette home.
        self._exec("adb shell input keyevent %s" % keycode_home)

    @need('bluetooth')
    def bt_opp_get_files_checksum(self, folder_name, file_list):
        """
        Returns a dict {file_name, checksum} for each file in file_list
        :type folder_name: str
        :param folder_name: the folder to search the files in.
                            If None or empty uses default folder
        :type file_list: str array
        :param file_list: list of file names
        :return: dict containing for each file its checksum
        """

        if not folder_name:
            # Default folder for bluetooth files
            folder_name = "/sdcard/bluetooth"

        result = {}
        version = self._exec("adb shell getprop ro.build.version.release")
        for file_name in file_list:
            if version.startswith('7'):
                cmd = "adb shell md5sum %s" % posixpath.join(folder_name, file_name)
            else:
                cmd = "adb shell md5 %s" % posixpath.join(folder_name, file_name)
            response = self._exec(cmd)

            if not response or "not found" in response:
                cmd = "adb shell md5sum %s" % posixpath.join(folder_name, file_name)
                response = self._exec(cmd)

            if not response or ("No such file or directory" in response):
                msg = "File %s not found" % file_name
                self._logger.error(msg)
                raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

            # response is in the form "<checksum> <filename>"
            # so split it and take the first item
            tokens = response.split()
            result[file_name] = tokens[0]

        return result

    @need('bluetooth')
    def connect_bt_device(self, remote_device_addr, profile):
        """
        Connects to a remote Bluetooth device profile.

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :type profile: str
        :param profile: profile to connect with like hsp, a2dp, nap...

        :rtype: boolean
        :return: connection status (true for connection succeeded)
        """
        self._logger.info("Trying to connect to Bluetooth %s profile" % profile)

        method = "connectProfile"

        profile = str(profile).upper()
        if profile not in (BtProfile.A2DP, BtProfile.HSP, BtProfile.PAN, BtProfile.HID):
            self._logger.error("not supported profile '%s'" % profile)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s: not supported profile %s" % (method, profile))

        args = "--es Address %s --es Profile %s" % (remote_device_addr, profile)

        # Launch connect process
        result = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        # Check correct profile has been connected successfully
        if BtProfile.NAME in result:
            if result[BtProfile.NAME] != profile:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "%s: Bad profile returned %s" % (method, result[BtProfile.NAME]))
        else:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "%s: No profile returned" % method)

        if BtConState.NAME in result:
            # Check connection succeeded
            if result[BtConState.NAME] == BtConState.CONNECTED:
                return True
            elif result[BtConState.NAME] == BtConState.DISCONNECTED:
                return False
            else:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "%s: Bad connection return value %s" % (method, result[BtConState.NAME]))
        else:
            self._logger.info("Connection State not found")
            if "output" in result:
                msg = "%s ERROR: %s" % (method, result["output"])
            else:
                msg = "%s ERROR" % method
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

    @need('bluetooth')
    def connect_bt_hid_device(self, remote_device_addr, reconnect="off"):
        """
        Connects Bluetooth hid device.

        :type reconnect: str or int
        :param reconnect: can be ('on', '1', 1) to enable
                                 ('off',' 0', 0) to disable

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :return: None
        """
        output = "%s not implemented on Android" % get_method_name()
        self._logger.warning(output)
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, output)

    @need('bluetooth')
    def get_bt_connection_state(self, remote_device_addr, profile):
        """
        Get the Bluetooth connection state to a specific profile.

        :type profile: str
        :param profile: profile to connect with like hsp, a2dp...

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :rtype: int
        :return: the connection state
                - BtConState.d[BtConState.DISCONNECTED] for disconnected
                - BtConState.d[BtConState.CONNECTED] for connected
                - BtConState.d[BtConState.CONNECTING] for connecting
                - BtConState.d[BtConState.DISCONNECTING] for disconnecting
        """
        self._logger.info("Trying to get connection state for Bluetooth %s profile" % profile)

        con_stat_res = BtConState.d[BtConState.UNKNOWN]

        method = "getProfileConnectionState"

        profile = str(profile).upper()
        if profile not in (BtProfile.A2DP, BtProfile.HSP, BtProfile.HID, BtProfile.PAN):
            self._logger.error("not supported profile '%s'" % profile)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s: not supported profile %s" % (method, profile))

        args = "--es Address %s --es Profile %s" % (remote_device_addr, profile)

        output = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        if BtConState.NAME in output:
            self._logger.info("Bluetooth profile %s is %s" % (profile, output[BtConState.NAME]))
            # Check connection succeeded
            if output[BtConState.NAME] not in BtConState.d:
                msg = "%s: Bad connection return value %s" % (method, output[BtConState.NAME])
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
            elif output[BtConState.NAME] == BtConState.UNKNOWN:
                msg = "unknown connection state"
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

            con_stat_res = BtConState.d[output[BtConState.NAME]]

        else:
            self._logger.info("Connection State not found")
            if "output" in output:
                msg = "%s ERROR: %s" % (method, output["output"])
            else:
                msg = "%s ERROR" % method
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

        return con_stat_res

    @need('bluetooth')
    def disconnect_bt_device(self, remote_device_addr, profile):
        """
        Disconnects from a remote Bluetooth device profile.

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :type profile: str
        :param profile: profile to disconnect with like hsp, a2dp, pan...

        :rtype: boolean
        :return: disconnection status (true for disconnection succeeded)
        """
        self._logger.info("Trying to disconnect from Bluetooth %s profile" % profile)

        disc_stat = False

        method = "disconnectProfile"
        profile = str(profile).upper()
        if profile not in (BtProfile.A2DP, BtProfile.HSP, BtProfile.PAN, BtProfile.HID):
            self._logger.error("not supported profile '%s'" % profile)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s: not supported profile %s" % (method, profile))

        args = "--es Address %s --es Profile %s" % (remote_device_addr, profile)

        # Launch disconnect process
        result = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        # Check correct profile has been disconnected successfully
        if BtProfile.NAME in result:
            if result[BtProfile.NAME] != profile:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "%s: Bad profile returned %s" % (method, result[BtProfile.NAME]))
        else:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                  "%s: No profile returned" % method)

        if BtConState.NAME in result:
            # Check disconnection succeeded
            if result[BtConState.NAME] == BtConState.DISCONNECTED:
                disc_stat = True
            elif result[BtConState.NAME] == BtConState.CONNECTED:
                disc_stat = False
            else:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "%s: Bad connection return value %s" % (method, result[BtConState.NAME]))
        else:
            self._logger.info("Connection State not found")
            if "output" in result:
                msg = "%s ERROR: %s" % (method, result["output"])
            else:
                msg = "%s ERROR" % method
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

        return disc_stat

    @need('bluetooth')
    def get_bt_audio_state(self, remote_device_addr, profile):
        """
        Get the Bluetooth audio state to a specific profile.

        :type profile: str
        :param profile: profile to use like hsp, a2dp...

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :rtype: int
        :return: the audio state
                - BtAudioState.d[BtAudioState.STOPPED] for stopped
                - BtAudioState.d[BtAudioState.PLAYING] for playing
        """
        self._logger.info("Trying to get audio state for Bluetooth %s profile" % profile)

        method = "getAudioState"
        profile = str(profile).upper()
        if profile not in (BtProfile.A2DP, BtProfile.HSP):
            self._logger.error("not supported profile '%s'" % profile)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s: not supported profile %s" % (method, profile))

        args = "--es Address %s --es Profile %s" % (remote_device_addr, profile)

        output = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        if BtAudioState.NAME in output:
            self._logger.info("Bluetooth audio for profile %s is %s" % (profile, output[BtAudioState.NAME]))
            # Check getaudioState succeeded
            if output[BtAudioState.NAME] not in BtAudioState.d.keys():
                msg = "%s: Bad audio state return value %s" % (method, output[BtAudioState.NAME])
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

        else:
            self._logger.info("Audio State not found")
            if "output" in output:
                msg = "%s ERROR: %s" % (method, output["output"])
            else:
                msg = "%s ERROR" % method
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

        return BtAudioState.d[output[BtAudioState.NAME]]

    @need('bluetooth')
    def is_bt_audio_connected(self):
        """
        Check whether BT audio device is connected or not

        :rtype: boolean
        :return: return True if BT audio device exists.
        """
        cmd = 'adb shell "dumpsys audio | grep mBluetoothName"'
        return_code, output = self._device.run_cmd(cmd, timeout=10)

        if return_code != Global.SUCCESS:
            msg = "uecmd is_bt_audio_connected: Error '%s' when trying to check bluetooth audio" % output
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        re_match = re.match(".*mBluetoothName=(.*)", output)
        if re_match and len(re_match.groups()) == 1:
            return True, re_match.group(1)
        else:
            return False, "None"

    @need('bluetooth', False)
    def set_bt_power(self, mode):
        """
        Sets the Bluetooth power.

        :type mode: str or int
        :param mode: can be ('on', '1', 1, 'STATE_ON') to enable
                            ('off', '0', 0, 'STATE_OFF') to disable

        :return: None
        """
        # pylint: disable=E1101
        if mode in ("on", "1", 1, str(BT_STATE.STATE_ON)):
            mode_verbose = str(BT_STATE.STATE_ON)
            mode = 1
        elif mode in ("off", "0", 0, str(BT_STATE.STATE_OFF)):
            mode_verbose = str(BT_STATE.STATE_OFF)
            mode = 0
        else:
            self._logger.error("set_bt_power : Parameter mode %s not valid" % mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid !")

        current_mode = str(self.get_bt_power_status())

        if current_mode == mode_verbose:
            self._logger.info("BT Power already set to %d" % mode)
        else:
            self._logger.info("Settings BT Power to %d" % mode)

            method = "setPower"
            args = "--es state %d" % mode

            self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

            self._logger.info("BT power set successfully")

    @need('bluetooth', False, str(BT_STATE.STATE_OFF))
    def get_bt_power_status(self):
        """
        Gets the Bluetooth power status.

        :rtype: str
        :return: BT_STATE ('STATE_OFF', 'STATE_ON', 'STATE_TURNING_OFF',
                     'STATE_TURNING_ON')
        """

        method = "getPower"

        power_status = int(self._internal_exec_v2(self._BLUETOOTH_MODULE,
                                                  method, is_system=True)["Power_state"])
        # pylint: disable=E1101
        if power_status == BtState.STATE_OFF:
            result = str(BT_STATE.STATE_OFF)
        elif power_status == BtState.STATE_ON:
            result = str(BT_STATE.STATE_ON)
        elif power_status == BtState.STATE_TURNING_OFF:
            result = str(BT_STATE.STATE_TURNING_OFF)
        elif power_status == BtState.STATE_TURNING_ON:
            result = str(BT_STATE.STATE_TURNING_ON)
        else:
            return_msg = "BT_STATE : invalid state value " + str(power_status)
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, return_msg)

        self._logger.info("BT power status: " + str(result))
        return str(result)

    @need('bluetooth')
    def bt_power_state_reached(self, expected_state, timeout=None):
        """
        Waits until the requested power state is reached (or timeout expires)
        :type expected_state: str
        :param expected_state: the state to wait for
        :type timeout: int
        :param timeout: maximum time to wait for end of transition (in seconds)
        :rtype: bool
        :return: True is state has been reached, False otherwise
        """
        try:
            self.get_bt_power_status_eot(expected_state, timeout)
        except DeviceException:
            self._logger.error("Expected state %s not reached before timeout" % expected_state)
            return False

        return True

    @need('bluetooth')
    def get_bt_power_status_eot(self, expected_state=None, timeout=None):
        """
        Wait for the end of transition to get the Bluetooth power status.
        If expected_state is passed, waits until that state is reached (or timeout expires)

        :type expected_state: str
        :param expected_state: the state to wait for
        :type timeout: int
        :param timeout: maximum time to wait for end of transition (in seconds)

        :rtype: str
        :return: BT_STATE ('STATE_OFF', 'STATE_ON')
        """

        # pylint: disable=E1101

        if timeout is None:
            timeout = self._uecmd_default_timeout

        if expected_state:
            self._logger.debug("Waiting for %s status" % expected_state)

        states = [expected_state] if expected_state else [str(BT_STATE.STATE_OFF), str(BT_STATE.STATE_ON)]
        power_status = ""
        t_0 = time.time()
        keep_looping = True
        while keep_looping:
            power_status = self.get_bt_power_status()
            keep_looping = power_status not in states and (time.time() - t_0 < timeout)
            if keep_looping:
                time.sleep(self._WAIT_FOR_POLL_SECS)

        if power_status not in states:
            return_msg = "power state transiton timeout:" + str(power_status)
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)

        return power_status

    @need('bluetooth')
    def set_bt_pairable(self, mode):
        """
        Switchs the Bluetooth device to pairable mode.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        self._logger.warning("%s not implemented on Android" %
                             get_method_name())

    @need('bluetooth')
    def get_bt_pairable_status(self):
        """
        Returns the Bluetooth device pairable status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        self._logger.warning("%s not implemented on Android" %
                             get_method_name())

    @need('bluetooth')
    def set_bt_discoverable(self, mode, timeout):
        """
        Sets the device discoverable.

        :type mode: str or int
        :param mode: can be ('noscan', 'none', SCAN_MODE_NONE) for no scan
          ('inquiry') for disoverable not connectable. Not supported by android
          ('page', 'connectable', SCAN_MODE_CONNECTABLE) for connectable
          ('both', SCAN_MODE_CONNECTABLE_DISCOVERABLE) to discoverable
        :type timeout: int
        :param timeout: can be 0 for Never,
                               120 for 2 minutes
                               300 for 5 minutes
                               3600 for 1 hour

        :return: None
        """
        # pylint: disable=E1101
        if mode in ('noscan', 'none', ScanMode.SCAN_MODE_NONE):
            mode = ScanMode.SCAN_MODE_NONE
            mode_verbose = BT_SCAN_MODE.SCAN_MODE_NONE
        elif mode in ('page', "connectable", ScanMode.SCAN_MODE_CONNECTABLE):
            mode = ScanMode.SCAN_MODE_CONNECTABLE
            mode_verbose = BT_SCAN_MODE.SCAN_MODE_CONNECTABLE
        elif mode in ("both", ScanMode.SCAN_MODE_CONNECTABLE_DISCOVERABLE):
            mode = ScanMode.SCAN_MODE_CONNECTABLE_DISCOVERABLE
            mode_verbose = BT_SCAN_MODE.SCAN_MODE_CONNECTABLE_DISCOVERABLE
        elif mode in ('inquiry'):
            msg = "set_bt_discoverable: Android does not support %s mode" % mode
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        else:
            self._logger.error("set_bt_discoverable: Parameter mode %s is not valid" % mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid!")

        if timeout not in (0, 120, 300, 3600):
            self._logger.error("set_bt_discoverable : Parameter timeout %d is not valid" % timeout)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid !")

        if mode == ScanMode.SCAN_MODE_CONNECTABLE_DISCOVERABLE:
            self._logger.info("Set BT discoverable timeout value to %d" % timeout)
            method = "openPopupSetDiscoverableTimeout"
            args = "--ei timeout %d" % (timeout)
            self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

            # Manually validate the popup - need to be changed when Android provide another method
            time.sleep(2.0);
            self._exec("adb shell input keyevent 20")
            self._exec("adb shell input keyevent 22")
            self._exec("adb shell input keyevent 66")

        self._logger.info("Set BT discoverable mode to %s" % mode_verbose)

        method = "setScanMode"
        args = "--es mode %d --es timeout %d" % (mode, timeout)

        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)
        self._logger.info("Set BT discoverable - success")

    @need('bluetooth')
    def get_bt_pairable_timeout(self):
        """
        Returns the timeout for the discoverable mode.

        :rtype: int
        :return: pairable timeout value
        """
        self._logger.warning("%s not implemented on Android" %
                             get_method_name())

    @need('bluetooth')
    def set_bt_autoconnect(self, mode):
        """
        Sets the Bluetooth autoconnect.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        self._logger.warning("%s not implemented on Android" %
                             get_method_name())

    @need('bluetooth')
    def get_bt_autoconnect_status(self):
        """
        Gets the Bluetooth autoconnect state.

        :return: None
        """
        self._logger.warning("%s not implemented on Android" %
                             get_method_name())

    @need('bluetooth')
    def set_bt_scanning(self, mode):
        """
        Starts/stops the device discovery session.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        self._logger.warning("%s not implemented on Android" %
                             get_method_name())

    @need('bluetooth')
    def pair_to_device(self, remote_device_addr, reconnect="off",
                       replyval=1, pincode="0000", passkey=0):
        """
        Connects to a Bluetooth remote device.

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address
                                   format 00:00:00:00:00:00
        :type reconnect: str or int
        :param reconnect: can be ('on', '1', 1) to enable
                                 ('off', '0', 0) to disable
        :type replyval: int
        :param replyval: pairing request reply value
                         1 = Positive reply
                         0 = Negative reply
        :type pincode: str
        :param pincode: Bluetooth device pincode for pairing process
                        (16 characters Max)
        :type passkey: int
        :param passkey: Bluetooth device passkey for pairing process
                       (6 digits Max)

        :rtype: list of 2 strings
        :return: (bond_state 'BOND_BONDED' or 'BOND_NONE') and
            (pinres is pin variant 'PIN' or 'PASSKEY' or 'REPLY' or 'NONE')
        """

        self._logger.info("Pairing to device : %s" %
                          remote_device_addr)

        self._validate_pairing_args(remote_device_addr, replyval, pincode, passkey)
        self._unpair_first_if_needed(remote_device_addr, reconnect)

        method = "pairDevice"
        args = "--es Address %s --ei PairRep %d --es Pincode %s --ei Passkey %d " \
        % (remote_device_addr, int(replyval), pincode, int(passkey))

        result = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        if BondState.NAME in result:
            # Check Pin variant used to pair
            # pylint: disable=E1101
            pinres = BT_PINVARIANT.ERROR
            if BtPinVariant.NAME in result \
               and result[BtPinVariant.NAME].isdigit():
                pinvar = int(result[BtPinVariant.NAME])
                if pinvar == BtPinVariant.PIN:
                    pinres = BT_PINVARIANT.PIN
                elif pinvar == BtPinVariant.PASSKEY:
                    pinres = BT_PINVARIANT.PASSKEY
                elif (pinvar in (BtPinVariant.PASSKEY_CONFIRMATION,
                                 BtPinVariant.CONSENT)):
                    pinres = BT_PINVARIANT.REPLY
                elif (pinvar in (BtPinVariant.DISPLAY_PASSKEY,
                                 BtPinVariant.DISPLAY_PIN,
                                 BtPinVariant.OOB_CONSENT)):
                    pinres = BT_PINVARIANT.NONE
                else:
                    pinres = BT_PINVARIANT.ERROR

            if str(result[BondState.NAME]).lower() == "none":
                return BT_BOND_STATE.BOND_NONE, pinres
            elif str(result[BondState.NAME]).lower() == "bonded":
                return BT_BOND_STATE.BOND_BONDED, pinres
            else:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "pairDevice: Bad return value %s" % (result[BondState.NAME]))
        else:
            self._logger.info("Bond_State not found")
            if "output" in result:
                msg = "pairDevice: Bad return value: %s" % (result["output"])
            else:
                msg = "pairDevice ERROR"
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

    @need('bluetooth')
    def unpair_bt_device(self, remote_device_addr):
        """
        Remove the remote Bluetooth device.

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address

        :return: None
        """
        self._logger.info("Unpairing from device %s" % remote_device_addr)

        method = "unpairDevice"
        args = "--es Address %s" % (str(remote_device_addr).upper())

        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

    @need('bluetooth')
    def wait_for_pairing(self, remote_device_addr, reconnect=0, replyval=1, pincode="0000", passkey=0, timeout=20):
        """
        Wait for a pairing request from a Bluetooth remote device.

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address
                                   format 00:00:00:00:00:00
        :type replyval: int
        :param replyval: pairing request reply value
                         1 = Positive reply
                         0 = Negative reply
        :type pincode: str
        :param pincode: Bluetooth device pincode for pairing process
                        (16 characters Max)
        :type passkey: int
        :param passkey: Bluetooth device passkey for pairing process
                       (6 digits Max)

        :type: timeout: int
        :param: timeout: timeout for expecting a pairing request

        :rtype: str
        :return: bond_state 'BOND_BONDED' or 'BOND_NONE' or 'BONDING'
        """

        self._logger.info("Wait for pairing to device : %s" % remote_device_addr)

        self._validate_pairing_args(remote_device_addr, replyval, pincode, passkey)

        self._unpair_first_if_needed(remote_device_addr, reconnect)

        method = "waitPairReq"
        args = "--es Address %s --ei PairRep %d --es Pincode %s --ei Passkey %d --ei timeout %d" \
        % (remote_device_addr, int(replyval), pincode, int(passkey), int(timeout))

        result = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        # pylint: disable=E1101
        if "output" in result:
            if str(result["output"]).lower() == "bonding":
                return BT_BOND_STATE.BOND_BONDING
            elif str(result["output"]).lower() == "bonded":
                return BT_BOND_STATE.BOND_BONDED
            else:
                msg = "waitPairReq ERROR: %s" % str(result["output"])
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
        else:
            msg = "waitPairReq ERROR"
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

    def _unpair_first_if_needed(self, remote_device_addr, reconnect):
        """
        Unpair if needed
        """
        if reconnect in ("on", "1", 1):  # if device is already paired, then force unpair
            reconnect = 1
            pairedlist = self.list_paired_device()
            for element in pairedlist:
                if str(element.address).upper() == remote_device_addr:
                    self.unpair_bt_device(remote_device_addr)
        elif not reconnect in ("off", "0", 0):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "reconnect : Bad parameter value %s" % (str(reconnect)))

    def _validate_bt_address(self, remote_device_addr):
        """
        Validate bt address
        """
        # Check address format
        remote_device_addr = str(remote_device_addr).upper()
        if not NetworkingUtil.is_valid_mac_address(remote_device_addr):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "BD address \'%s\' has a bad format!" \
                                     % remote_device_addr)

    def _validate_reply_arg(self, replyval):
        """
        Validate reply argument
        """
        # Check reply value
        if replyval not in [0, 1]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "BD replyval (%d) has a bad format!" \
                                     % replyval)

    def _validate_pincode_arg(self, pincode):
        """
        Validate pincode argument
        """
        if len(str(pincode)) > 16:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "pincode \'%s\' is too long (16 char Max)" \
                                     % pincode)

    def _validate_passkey_arg(self, passkey):
        """
        Validate passkey argument
        """
        if (len(str(passkey)) > 6) or (not str(passkey).isdigit()):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "passkey \'%s\' is too long (6 digits Max)" \
                                     % (str(passkey)))

    def _validate_pairing_args(self, remote_device_addr, replyval, pincode, passkey):
        """
        Validate pairing arguments
        """
        self._validate_bt_address(remote_device_addr)
        self._validate_reply_arg(replyval)
        self._validate_pincode_arg(pincode)
        self._validate_passkey_arg(passkey)

    @need('bluetooth')
    def wait_for_pairing_canceled(self):
        """
        Wait pairing cancelation due to timeout reached in embedded agent.

        :rtype: Boolean status of cancelation
        :return: True if pairing is successfully canceled False otherwise
        """
        result = self.listen_to_logcat(["status::FAILURE"],
                                       "Pairing time out expired",
                                       None,
                                       self._uecmd_default_timeout)

        # notify that scan is incomplete if listen_to_logcat status is false
        if not result[0]:
            self._logger.warning("Pairing canceled status not received from logcat")

        return result[0]

    @need('bluetooth')
    def list_paired_device(self):
        """
        Lists all paired devices

        :rtype: list of BluetoothDevice object
        :return: list of founded adapters (name/address)
        """
        self._logger.info("Retrieving list of paired devices")

        method = "listPairedDevice"
        device_list = []

        output = self._internal_exec_multiple_v2(self._BLUETOOTH_MODULE, method, is_system=True)

        for element in output:
            btdev = BluetoothDevice()
            if "Address" in element:
                self._logger.debug("PAIRED_DEVICE ADDRESS: %s" % element["Address"])
                btdev.address = element["Address"]
            if "Name" in element:
                self._logger.debug("PAIRED_DEVICE NAME: %s" % element["Name"])
                btdev.name = element["Name"]
            device_list.append(btdev)

        return device_list

    @need('bluetooth')
    def set_bt_authentication(self, interface, mode):
        """
        Sets the Bluetooth authentication to enabled/disabled for a given device

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        self._logger.warning("%s not implemented on Android" %
                             get_method_name())

    def set_agent_property(self, name, value):
        """
        Sets the properties of the simple agent file.

        :type name: str
        :param name: name of the property
        :type value: str
        :param value: value of the property

        :return: None
        """
        self._logger.warning("%s not implemented on Android" %
                             get_method_name())

    @need('bluetooth')
    def set_bt_default_link_policy(self, interface, mode):
        """
        Sets the Bluetooth default link policy for a given interface.

        .. warning:: This UeCmd requests root access on DUT

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...

        :type mode: str
        :param mode: "rswitch", "RSWITCH", "hold", "HOLD",
                     "sniff", "SNIFF", "park", "PARK"

        :return: None
        """
        self._logger.warning("This UeCmd requests root access on DUT")

        self._validate_bt_interface_arg(interface)

        if mode not in ("rswitch", "RSWITCH", "hold", "HOLD", "sniff", "SNIFF", "park", "PARK"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown mode (%s)" % mode)

        # Checks that interface exists.
        cmd = "adb shell hciconfig %s" % interface
        output = self._exec(cmd)

        if output.find(interface) == -1:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Interface not found!")

        # Set default link policy
        cmd = "adb shell hciconfig %s lp %s" % (interface, mode)
        output = self._exec(cmd)

        # Verify that the link policy is correctly set
        cmd = "adb shell hciconfig %s lp" % interface
        output = self._exec(cmd)
        if output.find(mode.upper()) == -1:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Fail to set bluetooth default link policy!")

    @need('bluetooth')
    def activate_bt_test_mode(self, interface):
        """
        Activates Bluetooth test mode.

        .. warning:: This UeCmd requests root access on DUT

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...

        :return: None
        """
        self._logger.warning("This UeCmd requests root access on DUT")

        self._validate_bt_interface_arg(interface)

        cmd = "adb shell hcitool -i %s cmd 0x06 0x03" % interface
        output = self._exec(cmd)
        if output != "":
            if output.find("Example:") != -1:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Fail to activate bluetooth  test mode!")

    @need('bluetooth')
    def set_bt_ctrl_event_mask(self, interface, mask):
        """
        Sets BT controller Event Mask.

        .. warning:: This UeCmd requests root access on DUT

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...

        :type mask: str
        :param mask: list of events which are generated by the HCI for the host.
                 the mask is made of hex values
                 ex : default mask = 0x00001FFFFFFFFFFF

        :return: None
        """
        self._logger.warning("This UeCmd requests root access on DUT")
        cmd = "adb shell hcitool -i %s cmd 0x03 0x0001" % interface
        mask = mask.replace('0x', "")

        for value in range(len(mask) / 2):
            cmd += " 0x" + mask[2 * value: 2 * value + 2]

        output = self._exec(cmd)
        if output != "":
            if output.find("Example:") != -1:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Fail to set bluetooth  controler event mask!")

    @need('bluetooth')
    def set_bt_ctrl_event_filter(self, interface,
                                 filter_type,
                                 filter_condition_type="",
                                 condition=""):
        """
        Sets BT controller Event Filter.

        .. warning:: This UeCmd requests root access on DUT

        :type interface: str
        :param interface: Bluetooth interface name, Eg. : hci0...

        :type filter_type: str
        :param filter_type: filter type , eg :
            - 0x00 Clear All Filters
            - 0x01 Inquiry Result
            - 0x02 Connection Setup
            - 0x03-0xFF Reserved for future use

        :type filter_condition_type: str
        :param filter_condition_type: For each Filter Type one or more Filter
        Condition types exists, eg :

            for 0x00 Clear All Filters:
                The Condition parameter is not used.

            for 0x01 Inquiry Result:
                0x00 Return responses from all devices during the Inquiry process.
                0x01 A device with a specific Class of Device responded to the Inquiry process.
                0x02 A device with a specific BD_ADDR responded to the Inquiry process.
                0x03-0xFF Reserved for future use

            for 0x02 Connection Setup:
                0x00 Allow Connections from all devices.
                0x01 Allow Connections from a device with a specific Class of Device.
                0x02 Allow Connections from a device with a specific BD_ADDR.
                0x03-0xFF Reserved for future use.

        :type condition: list of str
        :param condition: For each Filter Condition Type defined for the Inquiry Result Filter
        and the Connection Setup Filter, zero or more Condition parameters are
        required depending on the filter condition type and filter type.

        :return: None
        """
        self._logger.warning("This UeCmd requests root access on DUT")
        cmd = "adb shell hcitool -i %s cmd 0x03 0x0005 %s %s" % (interface, filter_type, filter_condition_type)
        if filter_condition_type != "":
            for element in condition:
                cmd += " %s" % element

        output = self._exec(cmd)
        if output != "":
            if output.find("Example:") != -1:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Fail to set bluetooth  controler event filter!")

    def _validate_bt_interface_arg(self, interface):
        """
        Validate interface argument
        """
        if not interface:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "interface must be valid (eg: hci0)")

    @need('bluetooth')
    def get_bt_adapter_address(self):
        """
        Gets Bluetooth address.

        :rtype: str
        :return: Bluetooth adapter address
        """
        method = "getAdapterAddress"
        address = str(self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)["Address"])
        return address


    def start_a2dp_media_player(self, file_name, timeout):
        """
        Starts A2DP media player setting the file to be played
        :type file_name: str
        :param file_name: The name of the mp3 file to be played
        :type timeout: int
        :param timeout: maximum time for the player to be active.
        """
        self._logger.info("Start A2DP Media Player for %s file; it will stay active for %d seconds"
                          % (file_name, timeout))
        method = "startA2dpPlayer"
        args = "--es fileName %s --ei timeout %d" % (file_name, timeout)
        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, timeout=timeout, is_system=True)

    def stop_a2dp_media_player(self):
        """
        Stops A2DP media player
        """
        self._logger.info("Stop A2DP Media Player")
        method = "stopA2dpPlayer"
        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)

    def control_a2dp_media_player(self, command):
        """
        Control A2DP media player
        :type command: str
        :param command: The command to execute on the player
        """
        self._logger.info("Control A2DP Media Player with command : %s" % command)
        method = "controlA2dpPlayer"
        args = "--es command %s" % command
        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

    @need('nfc', False, "OFF")
    def get_nfc_status(self):
        """
        Retrieve the NFC Status

        :rtype: str
        :return: Return the NFC status "OFF", "TURNING_ON", "ON", "TURNING_OFF", "UNKNOWN"
        """
        method = "statusNfc"
        status = self._internal_exec_v2(self._NFC_MODULE, method, is_system=True)["result"]

        if status == "1":
            return "OFF"
        elif status == "2":
            return "TURNING_ON"
        elif status == "3":
            return "ON"
        elif status == "4":
            return "TURNING_OFF"
        return "UNKNOWN"

    @need('nfc')
    def get_nfc_beam_status(self):
        """
        Retrieve the NFC BEAM Status

        :rtype: boolean
        :return: True if the NFC BEAM status is activated.
        """
        method = "getNfcBeamStatus"
        status = self._internal_exec_v2(self._NFC_MODULE, method, is_system=True)
        return status["result"] == "1"

    @need('nfc')
    def enable_nfc_beam(self):
        """
        Enable the NFC Beam. Auto activation is performed in order to modify NFC beam status.

        """
        method = "enableNfcBeam"
        self._internal_exec_v2(self._NFC_MODULE, method, is_system=True)

    @need('nfc')
    def disable_nfc_beam(self):
        """
        Disable the NFC Beam. Auto activation is performed in order to modify NFC beam status.

        """
        method = "disableNfcBeam"
        self._internal_exec_v2(self._NFC_MODULE, method, is_system=True)

    @need('nfc', False, "Device does not have nfc capability")
    def nfc_enable(self):
        """
        Enables NFC interface

        :return: None
        """
        method = "enableNfc"
        self._internal_exec_v2(self._NFC_MODULE, method, is_system=True)
        self._logger.info("NFC is switched on")

    @need('nfc')
    def force_nfc_state(self, state):
        """
        After a PhoneSystem.set_phone_lock(0) NFC is still OFF
        This function is a workarround to re-enable NFC after a set_phone_lock(0)

        :type state: integer or str
        :param state: 1 or on to force re-enable NFC
                      0 or off to get back to a normal mode.
        """
        if state in ("on", "1", 1):
            cmd = "adb shell am broadcast -a android.intent.action.USER_PRESENT"

        elif state in ("off", "0", 0):
            cmd = "adb shell am broadcast -a android.intent.action.SCREEN_OFF"

        self._exec(cmd)

    @need('nfc', False, "Device does not have nfc capability")
    def nfc_disable(self):
        """
        Disables NFC interface

        :return: None
        """
        method = "disableNfc"
        self._internal_exec_v2(self._NFC_MODULE, method, is_system=True)
        self._logger.info("NFC is switched off")

    @need('nfc')
    def check_nfc_crash(self, starttime):
        """
        Check the CrashInfo log to found "com.android.nfc" or "SIGSEGV" after
        the starttime

        :type starttime: float
        :param starttime: date after the one the check must be do
                            (get from time.time())

        :return: True if NFC Crash found, False if no NFC Crash found
        """
        crash_found = False

        if self._device_serial_number in (None, ""):
            msg = "Device must have a serial number to retrieve crash info"
            self._logger.error(msg)
            return True

        crash_info = CrashInfo(serial_number=self._device_serial_number)

        # Parse the CrashLog for potential NFC crash from starttime
        device_events = crash_info.get_event(detail_level=1)
        startime = datetime.fromtimestamp(starttime).strftime("%Y-%m-%d/%H:%M:%S")
        for event in device_events:
            if event["crashdir"] and (startime <= event["date"]):
                event_string = str(event)
                if ("com.android.nfc" in event_string) or ("SIGSEGV" in event_string):
                    msg = "Crash event found in CrashLog. %s" % event_string
                    self._logger.error(msg)
                    crash_found = True

        del crash_info

        return crash_found

    @need('nfc')
    def select_secure_element(self, secure_element):
        """
        Select NFC secure element

        :type secure_element: str
        :param secure_element: "DISABLED", "com.nxp.smart_mx.ID" or "com.nxp.uicc.ID"

        :return: None
        """
        self._logger.info("Secure element '%s' is being selected" % secure_element)
        method = "selectSecureElement"
        args = "--es secure_element %s" % secure_element
        self._internal_exec_v2(self._NFC_MODULE, method, args, is_system=True)

    @need('nfc')
    def read_nfc_tag(self):
        """
        Read data from NFC tag

        :rtype: String
        :return: read_data
        """
        self._logger.info("Read NFC tag")
        method = "readTag"
        read_data = self._internal_exec_v2(self._NFC_MODULE, method, is_system=True)
        return read_data["result"]

    @need('nfc')
    def write_nfc_tag(self, rtd_type, data):
        """
        Write data in tag

        :type rtd_type: str
        :param rtd_type: "RTD_TEXT", "RTD_SMARTPOSTER" or "RTD_URI"

        :type data: String
        :param data

        :return: None
        """
        self._logger.info("Write " + data + " in tag")
        method = "writeTag"
        args = "--es rtd_type '%s' --es data '%s'" % (rtd_type, data)
        self._internal_exec_v2(self._NFC_MODULE, method, args, is_system=True)

    def exchange_apdu_using_scapi(self, channel, reader, apdu_case, data_length, loop):
        """
        Exchange APDUs

        :type channel: str
        :param channel: LOGICAL, BASIC

        :type reader: String
        :param reader: UICC, SMX

        :type apdu_case: String
        :param apdu_case: ["1", "2", "3", "4", "ALL"]

        :type data_length: int
        :param data_length: [1..256]

        :type loop: int
        :param loop: number of commands sent during the test

        :return: None
        """

        if channel not in ("LOGICAL", "BASIC"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown channel (%s)" % channel)
        if reader not in ("UICC", "SMX"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown reader (%s)" % reader)
        if apdu_case not in ("1", "2", "3", "4", "ALL"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown APDU case")
        if data_length < 1 or data_length > 256:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid data length")

        self._logger.info("APDU case : " + apdu_case)
        self._logger.info("Channel : " + channel)
        self._logger.info("Reader : " + reader)
        self._logger.info("Data length : " + str(data_length))
        method = "exchangeApduUsingScapi"
        args = "--es channel %s " % channel
        args += "--es reader %s " % reader
        args += "--es apdu_case %s " % apdu_case
        args += "--ei data_length %d " % data_length
        args += "--ei loop %d" % loop
        self._internal_exec_v2(self._NFC_MODULE, method, args, is_system=True)

    @need('nfc')
    def set_default_nfc_p2p_configuration(self):
        """
        Set default P2P configuration
        Initiator : all passive and 424 active
        Target : all passive except 106, all active

        :return: None
        """

        initiator_conf = NfcP2pConfiguration.P2P_INITIATOR_CONFIGURATION["DEFAULT"]
        target_conf = NfcP2pConfiguration.P2P_TARGET_CONFIGURATION["DEFAULT"]

        method = "setNfcP2pConfiguration"
        args = "--ei initiatorMode %d --ei targetMode %d" % (initiator_conf, target_conf)
        self._internal_exec_v2(self._NFC_MODULE, method, args, is_system=True)

    @need('nfc')
    def set_nfc_p2p_configuration(self, mode, role, bitrate):
        """
        Set P2P configuration

        :type mode: str
        :param mode: ACTIVE, PASSIVE

        :type role: str
        :param role: INITIATOR, TARGET

        :type bitrate: str
        :param bitrate: 106, 212, 424

        :return: None
        """
        if role.upper() == "TARGET":
            initiator_conf = NfcP2pConfiguration.P2P_INITIATOR_CONFIGURATION["DISABLED"]
            if mode.upper() == "PASSIVE":
                target_conf = NfcP2pConfiguration.P2P_TARGET_CONFIGURATION["PASSIVE_ALL"]
            else:
                target_conf = NfcP2pConfiguration.P2P_TARGET_CONFIGURATION["ACTIVE_ALL"]
        else:
            target_conf = NfcP2pConfiguration.P2P_TARGET_CONFIGURATION["DISABLED"]
            if mode.upper() == "PASSIVE":
                if bitrate == "106":
                    initiator_conf = NfcP2pConfiguration.P2P_INITIATOR_CONFIGURATION["PASSIVE_106"]
                elif bitrate == "212":
                    initiator_conf = NfcP2pConfiguration.P2P_INITIATOR_CONFIGURATION["PASSIVE_212"]
                else:
                    initiator_conf = NfcP2pConfiguration.P2P_INITIATOR_CONFIGURATION["PASSIVE_424"]
            else:
                if bitrate == "106":
                    initiator_conf = NfcP2pConfiguration.P2P_INITIATOR_CONFIGURATION["ACTIVE_106"]
                elif bitrate == "212":
                    initiator_conf = NfcP2pConfiguration.P2P_INITIATOR_CONFIGURATION["ACTIVE_212"]
                else:
                    initiator_conf = NfcP2pConfiguration.P2P_INITIATOR_CONFIGURATION["ACTIVE_424"]

        method = "setNfcP2pConfiguration"
        args = "--ei initiatorMode %d --ei targetMode %d" % (initiator_conf, target_conf)
        self._internal_exec_v2(self._NFC_MODULE, method, args, is_system=True)

    @need('nfc')
    def nfc_touch_to_beam(self, screen_size, number_of_tap=10):
        """
        Start beam transfer touching the middle of the screen

        :type screen_size: str
        :param screen_size: 720x1184 format
        """
        self._logger.info("Touch screen to beam")
        screen_size = screen_size.split("x")

        x_tap = int(screen_size[0]) / 2
        y_tap = int(screen_size[1]) / 2

        cmd = "adb shell input touchscreen tap %d %d" % (x_tap, y_tap)

        for _ in range(int(number_of_tap)):
            self._exec(cmd)
            time.sleep(0.05)

    @need('bluetooth')
    def flush_bt_scanned_devices(self):
        """
        Clean BT scanned devices list
        """

        self._logger.info("Flush bt scanned devices")

        cmd = "adb shell am force-stop com.android.settings"

        self._exec(cmd)

    def _remove_bt_config_file(self):
        """
        Remove bt_conf.xml file
        For android release < JB_MR1, BT stack is BlueZ
        no such file to remove
        """
        return

    def get_default_addr(self):
        """
        Get the default BD address
        For android release < JB_MR1, BT stack is BlueZ
        Take DefaultBDAddress parameter from device catalog

        :rtype: str
        :return: default BD address. format: 12:34:56:78:9A:BC
                 otherwise return empty str
        """
        # Get Default BD address from device catalog file
        bdaddr = str(self._device.get_config("DefaultBDAddress"))
        bdaddr = str(bdaddr).upper().strip()
        self._logger.debug("Default BD Address is <%s>" % bdaddr)
        # May have X value for 'do not care'
        # Replace X value per 0 to check that format is valid
        bdaddr_z = bdaddr.replace('X', '0')
        if NetworkingUtil.is_valid_mac_address(bdaddr_z):
            return bdaddr
        else:
            return ""

    @need('bluetooth or wifi')
    def set_mac_addr(self, domain, mac_addr):
        """
        Set the mac address for the WIFI interface
        :type domain: String
        :param domain: can be 'wifi' or 'bt'
        :type mac_addr: String
        :param mac_addr: MAC address to set (format 08:00:28:44:44:44)

        :return: None
        """
        # Check domain
        if domain == "wifi":
            domain_idx = 10
        elif domain == "bt":
            domain_idx = 11
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Domain <%s> is not valid!" % domain)

        # Check address format
        if not NetworkingUtil.is_valid_mac_address(mac_addr):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "MAC address \'%s\' has a bad format!" % mac_addr)

        # Check provisioning method from device catalog
        prov_meth = str(self._device.get_provisioning_method()).upper()
        if prov_meth not in ["CC6_UMIP_ACCESS_APP", "TXEI_SEC_TOOLS"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "provisioning method <%s> is not valid!" % prov_meth)

        # Generate the MAC address in the device format and writes it into a temp file.
        tmp_file = tempfile.mkstemp(suffix="acs_mac")
        parsed_mac = mac_addr.split(":")
        bin_mac = struct.pack("BBBBBB", int(parsed_mac[0], 16), int(parsed_mac[1], 16),
                              int(parsed_mac[2], 16), int(parsed_mac[3], 16),
                              int(parsed_mac[4], 16), int(parsed_mac[5], 16))
        f = os.fdopen(tmp_file[0], "wb")

        f.write(bin_mac)
        f.close()

        dest_mac_file = str(self._device.get_provisioning_data_path() + "/macAddr.bin")
        # write MAC Address binary file to DUT file system
        self._device.push(tmp_file[1], dest_mac_file, 2)
        time.sleep(2)

        os.remove(tmp_file[1])

        # check presence of the file
        for cur_file in [dest_mac_file, ("/system/bin/" + prov_meth)]:
            file_exist = self._phone_system.check_file_exist_from_shell(cur_file)
            if not file_exist:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Can\'t find %s on target device" % cur_file)

        # Lock read/write access
        if prov_meth == "TXEI_SEC_TOOLS":
            self._logger.info("Lock read/write access")
            cmd = "adb shell %s -acd-lock" % prov_meth
            self._exec(cmd)
            write_cmd = prov_meth + " -acd-write"
        else:
            write_cmd = prov_meth + " write"

        # set the MAC Address on the device
        self._logger.info("setting mac address %s to the %s interface" % (mac_addr, domain))
        cmd = "adb shell %s %d %s 6 6" % (write_cmd, domain_idx, dest_mac_file)
        self._exec(cmd)

        if (domain == "bt") and (mac_addr == LocalConnectivityUtil.NULL_ADDRESS):
            self._remove_bt_config_file()

    def avrcp_expect_buttons(self, sequence, timeout, file_name):
        """
        Expects the given buttons sequence to be happening.
        @type sequence: String
        @param sequence: list of buttons semicolon separated
        @type timeout: int
        @param timeout: timeout for each button to be received
        @type file_name: String
        @param file_name: name of the audio file to be played

        @return: None
        """

        function = "expectButtons"
        args = ("--es buttons %s --es timeout %s --es fileName %s" \
                % (sequence, timeout, file_name))

        self._internal_exec_v2(self._BLUETOOTH_MODULE, function, args, is_system=True)

    @need('bluetooth')
    def set_bt_name(self, name):
        """
        Sets the device Bluetooth name.

        @type name: string
        @param name: name of the Bluetooth device
        @return: None
        """

        self._logger.info("Set BT Name to %s" % name)

        method = "setName"
        args = "--es Name %s" % name

        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)
        self._logger.info("Set BT Name - success")

    @need('bluetooth')
    def get_bt_device_name(self):
        """
        Sets the device Bluetooth name.

        @type name: string
        @param name: name of the Bluetooth device
        @return: None
        """

        self._logger.info("Getting BT Name for the current device")

        method = "getName"

        device_bt_name = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)['output']
        self._logger.info("Get BT Name - success: " + device_bt_name)
        return device_bt_name

    @need('bluetooth')
    def bluetooth_menu_settings(self):
        """
        Enter Bluetooth menu setting or exit from Bluetooth menu settings
        """
        self._exec("adb shell am start " + "-n com.android.settings/.bluetooth.BluetoothSettings")

    @need('bluetooth')
    def bt_start_opp_server(self):
        """
        Start Opp server
        """
        # DO NOTHING not need for android devices

    @need('bluetooth')
    def bt_stop_opp_server(self):
        """
        Stop Opp server
        """
        # DO NOTHING not need for android devices

    def set_ssp_mode(self, mode="on"):
        """
        Set Simple Secure Pairing mode to enable/disable it and force
        device to useLegacy Pairing mode instead.
        @type iface: string
        @param iface: bluetooth interface (hci0, hci1, etc.)
        @type mode: string
        @param mode: SSP mode: "off" => disabled, "on" => enabled
        @return None
        """
        cmd = "adb shell hciconfig " + self._bt_iface + " sspmode "
        if mode == "off":
            self._logger.info("Disable bluetooth SSP mode")
            cmd += "0"
        elif mode == "on":
            self._logger.info("Enable bluetooth SSP mode")
            cmd += "1"
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Mode <%s> is not valid!" % str(mode))

        self._exec(cmd)

    @need('bluetooth', False, str(BT_STATE.STATE_OFF))
    def get_bt_tethering_power_status(self):
        method = "getBluetoothTethering"
        power_status = str(self._internal_exec_v2(self._BLUETOOTH_MODULE,
                                                  method, is_system=True)["Pan_state"])
        # pylint: disable=E1101
        if power_status == "OFF":
            result = str(BT_STATE.STATE_OFF)
        elif power_status == "ON":
            result = str(BT_STATE.STATE_ON)
        elif power_status == "TURNING_OFF":
            result = str(BT_STATE.STATE_TURNING_OFF)
        elif power_status == "TURNING_ON":
            result = str(BT_STATE.STATE_TURNING_ON)
        else:
            return_msg = "BT_TETHERING_STATE : invalid state value " + str(power_status)
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, return_msg)

        self._logger.info("BT TETHERING status: " + str(result))
        return str(result)

    @need('bluetooth')
    def initiate_pair_to_device(self, remote_device_addr, reconnect="off",
                       replyval=1, pincode="0000", passkey=0):
        """
        Initiates connection to a Bluetooth remote device; expected state is BONDING

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address
                                   format 00:00:00:00:00:00
        :type reconnect: str or int
        :param reconnect: can be ('on', '1', 1) to enable
                                 ('off', '0', 0) to disable
        :type replyval: int
        :param replyval: pairing request reply value
                         1 = Positive reply
                         0 = Negative reply
        :type pincode: str
        :param pincode: Bluetooth device pincode for pairing process
                        (16 characters Max)
        :type passkey: int
        :param passkey: Bluetooth device passkey for pairing process
                       (6 digits Max)

        :rtype: list of 2 strings
        :return: (bond_state 'BOND_BONDED' or 'BOND_NONE') and
            (pinres is pin variant 'PIN' or 'PASSKEY' or 'REPLY' or 'NONE')
        """

        self._logger.info("Pairing to device : %s" %
                          remote_device_addr)

        self._validate_pairing_args(remote_device_addr, replyval, pincode, passkey)
        self._unpair_first_if_needed(remote_device_addr, reconnect)

        method = "initiatePairing"
        args = "--es Address %s --ei PairRep %d --es Pincode %s --ei Passkey %d " \
        % (remote_device_addr, int(replyval), pincode, int(passkey))

        result = self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

        if BondState.NAME in result:
            # Check Pin variant used to pair
            # pylint: disable=E1101
            pinres = BT_PINVARIANT.ERROR
            if BtPinVariant.NAME in result \
               and result[BtPinVariant.NAME].isdigit():
                pinvar = int(result[BtPinVariant.NAME])
                if pinvar == BtPinVariant.PIN:
                    pinres = BT_PINVARIANT.PIN
                elif pinvar == BtPinVariant.PASSKEY:
                    pinres = BT_PINVARIANT.PASSKEY
                elif (pinvar in (BtPinVariant.PASSKEY_CONFIRMATION,
                                 BtPinVariant.CONSENT)):
                    pinres = BT_PINVARIANT.REPLY
                elif (pinvar in (BtPinVariant.DISPLAY_PASSKEY,
                                 BtPinVariant.DISPLAY_PIN,
                                 BtPinVariant.OOB_CONSENT)):
                    pinres = BT_PINVARIANT.NONE
                else:
                    pinres = BT_PINVARIANT.ERROR

            if str(result[BondState.NAME]).lower() == "none":
                return BT_BOND_STATE.BOND_NONE, pinres
            elif str(result[BondState.NAME]).lower() == "bonded":
                return BT_BOND_STATE.BOND_BONDED, pinres
            elif str(result[BondState.NAME]).lower() == "bonding":
                return BT_BOND_STATE.BOND_BONDING, pinres
            else:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "pairDevice: Bad return value %s" % (result[BondState.NAME]))
        else:
            self._logger.info("Bond_State not found")
            if "output" in result:
                msg = "pairDevice: Bad return value: %s" % (result["output"])
            else:
                msg = "pairDevice ERROR"
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
