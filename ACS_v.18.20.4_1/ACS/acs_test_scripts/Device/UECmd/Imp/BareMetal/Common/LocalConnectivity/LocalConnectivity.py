"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW
:summary: This file implements LocalConnectivity UECmds for Linux device
:since: 04/29/2014
:author: jreynaux
"""
import inspect
import re
import time
from datetime import datetime, timedelta

from UtilitiesFWK.Utilities import FINDKEY, internal_shell_exec, str_to_bool_ex
from acs_test_scripts.Device.UECmd.UECmdTypes import BtProfile, BtServiceClass, BluetoothDevice,\
    BtGattNotification, BT_BOND_STATE, BT_STATE
from acs_test_scripts.Device.UECmd.Interface.LocalConnectivity.ILocalConnectivity import ILocalConnectivity
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.Imp.BareMetal.Common.Base import Base
from acs_test_scripts.Device.UECmd.UECmdDecorator import need


class LocalConnectivity(Base, ILocalConnectivity):
    """
    Class that handles all local connectivity related operations.

    /!\ IMPORTANT NOTE /!\
    Please note that this device type is a BLE device that mean
    most bluetooth operations are initiated from a remote device with a test app.
    e.g. connection, service discovering etc ...
    """

    @need("bluetooth")
    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        ILocalConnectivity.__init__(self, device)
        self._logger = device.get_logger()
        self._device = device

    def bt_reset_device(self):
        """
        Resets DUT Bluetooth adapter. Turn OFF then ON and check power state is successfully ON

        :return: None
        """
        self._logger.debug("{0}: Not yet implemented on {1} device".
                           format(inspect.stack()[0][3], self._device.get_name()))

    def get_bt_adapter_address(self):
        """
        Gets Bluetooth address of DUT.

        :raise DeviceException if unable to get device bt address

        :rtype: str
        :return: Bluetooth adapter address
        """
        self._logger.debug("Getting Bluetooth MAC address")

        rv = self._internal_exec(cmd="ble_get_ids", broadcast=False)
        try:
                bt_address = "%02X:%02X:%02X:%02X:%02X:%02X" % tuple(map(lambda x: ord(x), tuple(rv[0])))
        except Exception:
            error_message = "Unable to get BT address ({0})".format(rv)
            self._logger.error(error_message)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_message)

        self._logger.debug("Device bt address: {0}".format(bt_address))
        return bt_address

    def get_bt_device_name(self):
        """
        Gets Bluetooth Name of DUT.

        :raise DeviceException if unable to get device bt address

        :rtype: str
        :return: Bluetooth device name
        """
        self._logger.debug("Getting Bluetooth DUT Name")

        rv = self._internal_exec(cmd="ble_get_ids", broadcast=False)

        if len(rv) != 3 or rv[2] == "":
            error_message = "Unable to get BT name ({0})".format(rv)
            self._logger.warning(error_message)
            return ""

        bt_name = str(rv[2])
        self._logger.debug("Device bt name: {0}".format(bt_name))
        return bt_name

    def connect_bt_device(self, remote_device_addr, profile=BtProfile.GATT):
        """
        Connects to a remote Bluetooth device profile.

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :type profile: str
        :param profile: profile to connect with like hsp, a2dp...

        :rtype: boolean
        :return: connection status (true for connection succeeded)
        """
        self._logger.debug("Connecting to device: {0}".format(remote_device_addr))

        # Check if not already connected
        connected_devices = self.list_connected_device()
        for device in connected_devices:
            if str(remote_device_addr).upper() in str(device.address).upper():
                self._logger.warning('Device already connected !')
                return True

        action = "connect"
        param = str(remote_device_addr)
        device_name = self.get_bt_device_name()
        if device_name not in [None, ""]:
            cmd = "-a com.acs.action.{0} -e DEVICE_ADDRESS {1} -e DEVICE_NAME {2}".\
                format(action, param, device_name)
        else:
            cmd = "-a com.acs.action.{0} -e DEVICE_ADDRESS {1}".format(action, param)

        cmd += " --ez AUTOCONNECT true"

        try:
            self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)
            connected = True
        except DeviceException as de:
            self._logger.error(de)
            connected = False

        return connected

    def disconnect_bt_device(self, remote_device_addr, profile=BtProfile.GATT):
        """
        Disconnects from a remote Bluetooth device profile.

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :type profile: str
        :param profile: profile to disconnect with like hsp, a2dp...

        :rtype: boolean
        :return: disconnection status (true for disconnection succeeded)
        """
        self._logger.debug("Disconnecting from connected device")

        action = "disconnect"
        cmd = "-a com.acs.action.{0}".format(action)
        try:
            self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)
            status = True
        except DeviceException as de:
            self._logger.error(de)
            status = False

        return status

    def pair_to_device(self, remote_device_addr, reconnect="off", replyval=1, pincode="0000", passkey=0):
        """
        Pairs a DUT to Bluetooth remote device.

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
        (pinRes is pin variant 'PIN' or 'PASSKEY' or 'REPLY' or 'NONE')
        """
        output = [BT_BOND_STATE.BOND_NONE, 'NONE']

        # Check if not already paired
        paired_devices = self.list_paired_device()
        for device in paired_devices:
            # If device paired do
            if str(remote_device_addr).upper() in str(device.address).upper():
                # if reconnect = on, mean unpair first
                if reconnect == 'on':
                    self._logger.warning('Unpair device first... ')
                    self.unpair_bt_device(remote_device_addr)
                    output = [BT_BOND_STATE.BOND_NONE, 'NONE']
                else:
                    self._logger.warning('Device already paired !')
                    output = [BT_BOND_STATE.BOND_BONDED, 'NONE']

                break

        # If not bonded, pair
        self._logger.debug("Pairing to device: {0}".format(remote_device_addr))
        if output[0] == BT_BOND_STATE.BOND_NONE:
            # Warning: this UeCmd will now explicitly need to connect first
            action = "pair"
            cmd = "-a com.acs.action.{0} -e DEVICE_ADDRESS {1}".format(action, remote_device_addr)

            output = self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)
            if output is not None:
                output = [BT_BOND_STATE.BOND_BONDED, 'NONE']

        return output

    def unpair_bt_device(self, remote_device_addr):
        """
        Remove the remote Bluetooth device.

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address

        :return: None
        """
        self._logger.debug("Unpairing from device {0}".format(remote_device_addr))
        action = "unpair"
        cmd = "-a com.acs.action.{0} -e DEVICE_ADDRESS {1}".format(action, remote_device_addr)
        self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)

    def bt_scan_devices(self):
        """
        Scans remote Bluetooth devices.

        :rtype: list of BluetoothDevice object
        :return: list of founded adapters (name/address)
        """
        default_timeout = 5000
        action = "scan"
        cmd = "-a com.acs.action.{0} -e TIME_OUT {1}".format(action, default_timeout)
        device_list = []

        output = self._internal_exec_multiple(cmd=cmd, timeout=default_timeout,
                                              remove_triggered_message=True, broadcast=True, instrument=False)

        for element in output:
            btdev = BluetoothDevice()
            if "device_address" in element:
                btdev.address = element["device_address"]
            if "device_name" in element:
                btdev.name = element["device_name"]
            device_list.append(btdev)
            self._logger.debug("Visible device found, name: {0}, address: {1}".format(btdev.name, btdev.address))

        return device_list

    def bt_fota(self, mac_address, files_to_flash, timeout=300):
        """
        Do a FOTA (Flash Over The Air) using Bluetooth.
        The FOTA is not donne directly by the host computer. Instead it is done by the
        BLE test app of the android device connected to the host computer

        An exception is raised if an error occurs during the FOTA process (i.e. if
        this method ends, the FOTA has been done correctly).

        :type mac_address: str
        :param mac_address: MAC address of the device on which the FOTA shall be done
        :type files_to_flash: list
        :param files_to_flash: list of files to flash with FOTA. NOt used for clark because
                                files to flash are predefined
        :type timeout: int
        :param timeout: FOTA timeout value

        :return: None
        """

        self._logger.info("Do FOTA on device {0} with files : {1}".format(mac_address, files_to_flash))
        # Remove all potential previous files on the android device
        internal_shell_exec(cmd="adb shell rm /sdcard/Pictures/*", timeout=10, silent_mode=True)

        # Copy files to flash on android device
        for file_tmp in files_to_flash:
            self._logger.info("Treating {0}".format(file_tmp))
            internal_shell_exec(cmd="adb push {0} /sdcard/Pictures/".format(file_tmp), timeout=10, silent_mode=True)

        # Launch FOTA using clark test app
        self._logger.info("Launch FOTA using clark test app")
        cmd = "-a com.acs.action.fota -e DEVICE_ADDRESS {0}".format(mac_address)
        if len(files_to_flash) > 1:
            cmd += " -e PARTITION all"
            self._logger.debug("Flash all partition")
        elif "sdbootreco" in files_to_flash[0]:
            cmd += " -e PARTITION sdbootreco"
            self._logger.debug("Flash sdbootreco only")
        elif "application" in files_to_flash[0]:
            cmd += " -e PARTITION application"
            self._logger.debug("Flash app only")
        else:
            cmd = "-a com.acs.action.fw_update -e DEVICE_ADDRESS {0}".format(mac_address)
            self._logger.debug("Warning: unknwon partition to flash. Using fw_update intent instead of fota intent")

        self._internal_exec(cmd=cmd, timeout=timeout)

    def set_bt_discoverable(self, mode="both", timeout=5):
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
        self._logger.debug("Setting DUT discoverable (advertise mode)")
        self._internal_exec(cmd="ble_advertise", broadcast=False)

    def bt_service_browsing(self, bd_address, class_to_browse=BtProfile.GATT):
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
        # TODO: Enhance method to respect docstring, can use com.acs.action.get_services
        self._logger.debug("Discovering BLE services ...")

        # Trigger service discovering
        action = "discover_services"
        cmd = "-a com.acs.action.{0}".format(action)
        self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)

        # Then get service list
        action = "get_services"
        cmd = "-a com.acs.action.{0}".format(action)
        output = self._internal_exec_multiple(cmd=cmd, timeout=self._uecmd_default_timeout,
                                              remove_triggered_message=True,
                                              broadcast=True, instrument=False)
        btdev = BluetoothDevice()
        btdev.address = bd_address
        btdev.name = ""
        # Try to get device name, may be not possible if no UART used
        try:
            btdev.name = self.get_bt_device_name()
        except AcsConfigException as ace:
            self._logger.debug('Unable to get BT device name ...({0})'.format(ace))

        # Build simple service list from output
        service_list = []
        for element in output:
            if "S_UUID" in element.keys():
                service_list.append(element["S_UUID"])
        # Remove duplicate
        service_list = list(set(service_list))
        # Get name from BtServiceClass.GATT_SERVICE if possible
        service_list_named = []
        for service in service_list:
            l = FINDKEY(BtServiceClass.GATT_SERVICE, service)
            if len(l) > 0:
                service_list_named.append(l[0])
            else:
                service_list_named.append(service)

        btdev.uuids = service_list_named

        return True, btdev

    def bt_gatt_read_characteristic(self, service_name, char_name, data_type="string"):
        """
        Read a GATT Bluetooth characteristic on device

        :type service_name:  str
        :param service_name: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.
                            Can also be direct uuid to a characteristic

        :type char_name:  str
        :param char_name: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.
                        Can also be direct uuid to a characteristic

        :type: data_data_type: str
        :param data_type: "string', "int", "hexa", "hexa_endian", default "string"
                            if string: convert hexa to string before return value
                            if int: convert hexa to integer before return value
                            if hexa: no conversion will be operated, only concatenation.
                            if hexa_endian: will convert as little endian and concatenate before return value.

        :return: str
        """
        self.__check_gatt_service_characteristic(service_name, char_name)

        self._logger.debug("Reading '{0}' characteristic on '{1}' service".format(char_name, service_name))

        # Log only if recognized service and characteristic
        if service_name in BtServiceClass.GATT_SERVICE.keys():
            self._logger.debug("'{0}' service UUID: {1}"
                               .format(service_name, BtServiceClass.GATT_SERVICE[service_name]))
        if char_name in BtServiceClass.GATT_CHARACTERISTIC.keys():
            self._logger.debug("'{0}' characteristic UUID: {1}"
                               .format(char_name, BtServiceClass.GATT_CHARACTERISTIC[char_name]))

        # If key given then get value, otherwise use given UUID instead
        s_uuid = BtServiceClass.GATT_SERVICE[service_name]\
            if service_name in BtServiceClass.GATT_SERVICE.keys() else service_name
        c_uuid = BtServiceClass.GATT_CHARACTERISTIC[char_name] \
            if char_name in BtServiceClass.GATT_CHARACTERISTIC.keys() else char_name

        action = "read"
        cmd = "-a com.acs.action.{0} -e S_UUID {1} -e C_UUID {2}".format(action, s_uuid, c_uuid)
        output = self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)

        if output != "":
            value = output["value"]
            self._logger.debug("{0} characteristic raw value: {1}".format(char_name, value))
        else:
            raise DeviceException(DeviceException.DEFAULT_ERROR_CODE,
                                  "Unable to retrieve {0} characteristic ({1}) from {2} service ({3}".format(
                                      char_name, BtServiceClass.GATT_CHARACTERISTIC[char_name],
                                      service_name, BtServiceClass.GATT_SERVICE[service_name]))

        if "string" in data_type:
            # Convert given hex value to readable string
            value = "".join(value.split()).decode('hex')
            # Strip non printable characters
            value = ''.join([c for c in value if ord(c) > 31 or ord(c) == 9])
        elif "int" in data_type:
            # Convert given hex value to readable integer
            value = "{0}".format(int(value.replace(" ", ""), 16))
        elif "hexa_endian" in data_type:
            little_endian_hex = "".join(value.split())
            self._logger.debug('Retrieved data as little endian format:{0}'.format(little_endian_hex))
            self._logger.debug('Convert as big endian format')
            big_endian_list = []
            for i in range(0, len(little_endian_hex), 2):
                big_endian_list.insert(0, little_endian_hex[i:i+2])
            value = "".join(big_endian_list)
            self._logger.debug('Convert as big endian format: {0}'.format(value))
        elif "hexa" in data_type:
            self._logger.warning('Unconverted retrieved data !')
            value = "".join(value.split())

        self._logger.info("{0} characteristic value ({1}): {2}".format(char_name, data_type, value))
        return value

    def bt_gatt_write_characteristic(self, service_name, char_name, data, data_type="hexa", acknowledgement=True):
        """
        Write data on GATT Bluetooth characteristic on device

        :type data: str
        :param data: Data to be send to device, as str format

        :type: data_type: str
        :param data_type: "hexa", "hexa_endian" or "string", default "hexa"
                            If "hexa_endian" will convert as little endian before writing

        :type service_name:  str
        :param service_name: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.
                            Can also be direct uuid to a characteristic

        :type char_name:  str
        :param char_name: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.
                        Can also be direct uuid to a characteristic

        :type acknowledgement: bool
        :param acknowledgement: Whether wait for acknowledgement or not.

        :rtype: None
        """
        self.__check_gatt_service_characteristic(service_name, char_name)

        message = "Writing {0} ({1}) in {2} characteristic on {3} service" \
            .format(data, data_type, char_name, service_name)

        if not acknowledgement:
            message += " (no ack)"

        self._logger.debug(message)
        if acknowledgement:
            action = "write"
        else:
            action = "write_no_ack"

        if "hexa_endian" in data_type:
            self._logger.warning('Convert data as little endian format before sending !')
            little_endian_list = []
            for i in range(0, len(data), 2):
                little_endian_list.insert(0, data[i:i+2])
            data = "".join(little_endian_list)
            self._logger.debug('Converted data as little endian format: {0}'.format(data))
            # switch back data type "hexa" for test app
            data_type = "hexa"

        # If key given then get value, otherwise use given UUID instead
        s_uuid = BtServiceClass.GATT_SERVICE[service_name]\
            if service_name in BtServiceClass.GATT_SERVICE.keys() else service_name
        c_uuid = BtServiceClass.GATT_CHARACTERISTIC[char_name] \
            if char_name in BtServiceClass.GATT_CHARACTERISTIC.keys() else char_name

        cmd = "-a com.acs.action.{0} -e S_UUID {1} -e C_UUID {2} -e VALUE {3} -e TYPE {4}".format(action,
                                                                                                  s_uuid,
                                                                                                  c_uuid,
                                                                                                  data,
                                                                                                  data_type)
        if acknowledgement:
            self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)
        else:
            # no acknowledgment just send command without verification
            try:
                self._internal_exec(cmd=cmd, timeout=1)
            except DeviceException:
                return

    def __check_gatt_service_characteristic(self, service, characteristic):
        """
        Check if given value for ble service and characteristic complies to raw uuid value or
        name.

        :type service:  str
        :param service: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                        Basically they are named from the GATT service standard specification.
                        Can also be direct uuid to a characteristic

        :type characteristic:  str
        :param characteristic: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                               Basically they are named from the GATT characteristic standard specification.
                               Can also be direct uuid to a characteristic

        :rtype: None
        :raise AcsConfigException is wrong formatted or unknown name
        """
        regex = re.compile('^([0-9a-z]{8})-([0-9a-z]{4})-([0-9a-z]{4})-([0-9a-z]{4})-([0-9a-z]{12})$')

        service_name_match = regex.match(service)
        if service_name_match is not None:  # Valid service UUID format
            self._logger.debug('Will use direct UUID for service')
        elif service in BtServiceClass.GATT_SERVICE.keys():  # Valid service name format
            self._logger.debug("'{0}' service UUID: {1}".format(service, BtServiceClass.GATT_SERVICE[service]))
        else:  # Invalid service name or UUID
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unknown gatt service, allowed service value are UUID or {0}"
                                     .format(BtServiceClass.GATT_SERVICE.keys()))

        char_name_match = regex.match(characteristic)
        if char_name_match is not None:  # Valid characteristic UUID format
            self._logger.debug('Will use direct UUID for characteristic')
        elif characteristic in BtServiceClass.GATT_CHARACTERISTIC.keys():  # Valid characteristic name format
            self._logger.debug("'{0}' characteristic UUID: {1}".format(characteristic,
                                                                       BtServiceClass.GATT_CHARACTERISTIC[
                                                                           characteristic]))
        else:  # Invalid service name or UUID
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unknown gatt characteristic, allowed characteristics value are UUID or {0}"
                                     .format(BtServiceClass.GATT_CHARACTERISTIC.keys()))

    def list_connected_device(self):
        """
        Lists all connected devices

        :rtype: list of BluetoothDevice object
        :return: list of paired devices (name/address)
        """
        self._logger.debug("Retrieving list of paired devices")

        action = "list_connected_devices"
        device_list = []
        cmd = "-a com.acs.action.{0}".format(action)

        connected_devices = self._internal_exec_multiple(cmd=cmd, timeout=self._uecmd_default_timeout,
                                                         remove_triggered_message=True,
                                                         broadcast=True, instrument=False)
        for element in connected_devices:
            if 'multiple_output' in element and "0" in element["multiple_output"]:
                self._logger.debug('No device connected')
                break

            btdev = BluetoothDevice()
            if "device_address" in element:
                btdev.address = element["device_address"]

            if "device_name" in element:
                btdev.name = element["device_name"]
            if "bonded" in element:
                btdev.bonded = element["bonded"]

            self._logger.debug("Connected device found, name: {0}, address: {1}".format(btdev.name, btdev.address))
            device_list.append(btdev)

        return device_list

    def list_paired_device(self):
        """
        Lists all paired devices

        :rtype: list of BluetoothDevice object
        :return: list of paired devices (name/address)
        """
        self._logger.debug("Retrieving list of paired devices")

        action = "list_paired_devices"
        device_list = []
        cmd = "-a com.acs.action.{0}".format(action)

        paired_devices = self._internal_exec_multiple(cmd=cmd, timeout=self._uecmd_default_timeout,
                                                      remove_triggered_message=True,
                                                      broadcast=True, instrument=False)

        for element in paired_devices:
            if 'multiple_output' in element and "0" in element["multiple_output"]:
                self._logger.debug('No device paired')
                break

            btdev = BluetoothDevice()
            if "device_address" in element:
                btdev.address = element["device_address"]
            if "device_name" in element:
                btdev.name = element["device_name"]
            if "bonded" in element:
                btdev.bonded = element['bonded']
            self._logger.debug("Paired device found, name: {0}, address: {1}".format(btdev.name, btdev.address))
            device_list.append(btdev)

        return device_list

    def bt_gatt_subscribe_notification(self, service_name, char_name):
        """
        Subscribe to a GATT Bluetooth notification related to given service/characteristic on device

        :type service_name:  str
        :param service_name: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.
                            Can also be direct uuid to a service

        :type char_name:  str
        :param char_name: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.
                        Can also be direct uuid to a characteristic

        :return: str
        """
        self.__check_gatt_service_characteristic(service_name, char_name)

        self._logger.debug(
            "Subscribing to notification on '{0}' characteristic on '{1}' service".format(char_name, service_name))

        # Log only if recognized service and characteristic
        if service_name in BtServiceClass.GATT_SERVICE.keys():
            self._logger.debug("'{0}' service UUID: {1}"
                               .format(service_name, BtServiceClass.GATT_SERVICE[service_name]))
        if char_name in BtServiceClass.GATT_CHARACTERISTIC.keys():
            self._logger.debug("'{0}' characteristic UUID: {1}"
                               .format(char_name, BtServiceClass.GATT_CHARACTERISTIC[char_name]))

        # If key given then get value, otherwise use given UUID instead
        s_uuid = BtServiceClass.GATT_SERVICE[service_name]\
            if service_name in BtServiceClass.GATT_SERVICE.keys() else service_name
        c_uuid = BtServiceClass.GATT_CHARACTERISTIC[char_name] \
            if char_name in BtServiceClass.GATT_CHARACTERISTIC.keys() else char_name

        action = "subscribe_notification"
        cmd = "-a com.acs.action.{0} -e S_UUID {1} -e C_UUID {2}".format(action,
                                                                         s_uuid,
                                                                         c_uuid)
        # Get "now" opcode
        now_dt = datetime.now()
        now_dt_op_code = now_dt.strftime("%y%m%j%H%M")
        self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout, op_code=now_dt_op_code, remove_triggered_message=False)

    def bt_gatt_unsubscribe_notification(self, service_name, char_name):
        """
        Unsubscribe to a GATT Bluetooth notification related to given service/characteristic on device

        :type service_name:  str
        :param service_name: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.
                            Can also be direct uuid to a service

        :type char_name:  str
        :param char_name: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.
                        Can also be direct uuid to a characteristic

        :return: str
        """
        self.__check_gatt_service_characteristic(service_name, char_name)

        self._logger.debug(
            "Unsubscribing to notification on '{0}' characteristic on '{1}' service".format(char_name, service_name))

        # Log only if recognized service and characteristic
        if service_name in BtServiceClass.GATT_SERVICE.keys():
            self._logger.debug("'{0}' service UUID: {1}"
                               .format(service_name, BtServiceClass.GATT_SERVICE[service_name]))
        if char_name in BtServiceClass.GATT_CHARACTERISTIC.keys():
            self._logger.debug("'{0}' characteristic UUID: {1}"
                               .format(char_name, BtServiceClass.GATT_CHARACTERISTIC[char_name]))

        # If key given then get value, otherwise use given UUID instead
        s_uuid = BtServiceClass.GATT_SERVICE[service_name]\
            if service_name in BtServiceClass.GATT_SERVICE.keys() else service_name
        c_uuid = BtServiceClass.GATT_CHARACTERISTIC[char_name] \
            if char_name in BtServiceClass.GATT_CHARACTERISTIC.keys() else char_name

        action = "unsubscribe_notification"
        cmd = "-a com.acs.action.{0} -e S_UUID {1} -e C_UUID {2}".format(action,
                                                                         s_uuid,
                                                                         c_uuid)
        self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)

    def bt_gatt_get_notification(self, service_name, char_name, time_lap):
        """
        Get GATT Bluetooth notifications related to given service/characteristic on device

        :type service_name:  str
        :param service_name: Name of service, can use UECmdTypes.BtServiceClass.GATT_SERVICE/
                            Basically they are named from the GATT service standard specification.
                            Can also be direct uuid to a service

        :type char_name:  str
        :param char_name: Name of characteristic, can use UECmdTypes.BtServiceClass.GATT_CHARACTERISTIC
                        Basically they are named from the GATT characteristic standard specification.
                        Can also be direct uuid to a characteristic

        :type time_lap: int
        :param time_lap: Lap of time where notification should be backlogged (in minutes)

        :rtype: list
        :return: List of UECmdTypes.BtGattNotification
        """
        self.__check_gatt_service_characteristic(service_name, char_name)

        self._logger.debug("Retrieve notification about '{0}' characteristic on '{1}' service, until {2} minutes ago".
                           format(char_name, service_name, time_lap))

        # Get "now" opcode
        now_dt = datetime.now()

        # Get notifications for each minutes back until time_lap
        self._logger.debug("First retrieve notification until {0} minutes ago".format(time_lap))
        notifications = []
        for minutes in range(time_lap):
            current_dt = now_dt - timedelta(minutes=minutes)
            # %y	Year without century as a zero-padded decimal number. (16)
            # %m	Month as a zero-padded decimal number. (01)
            # %d	Day of the month as a zero-padded decimal number. (07)
            # %H	Hour (24-hour clock) as a zero-padded decimal number. (15)
            # %M	Minute as a zero-padded decimal number. (50)
            current_dt_op_code = current_dt.strftime("%y%m%d%H%M")
            notifications += self.__extract_notifications_from_log(current_dt_op_code)

        # Filter on service/characteristic
        s_filter = service_name
        if service_name in BtServiceClass.GATT_SERVICE:
            # filter on service name -> translate to UUID
            s_filter = BtServiceClass.GATT_SERVICE.get(service_name)

        c_filter = char_name
        if char_name in BtServiceClass.GATT_CHARACTERISTIC:
            c_filter = BtServiceClass.GATT_CHARACTERISTIC.get(char_name)

        self._logger.debug("Then filter notif using service {0} and characteristic {1}".format(s_filter, c_filter))
        i = len(notifications)-1
        while i >= 0:
            if notifications[i].service not in s_filter or notifications[i].characteristic not in c_filter:
                self._logger.debug("Pop notif {0}: ({1} / {2}, type={3})".
                                   format(i, notifications[i].service,
                                          notifications[i].characteristic, notifications[i].type))
                notifications.pop(i)
            i -= 1

        self._logger.debug("Notifications list after filtering:")
        for index, notif in enumerate(notifications):
            self._logger.debug("- Notif {0}: {1} / {2}".format(index, notif.service, notif.characteristic))
            self._logger.debug("    type={0}, data: {1}".format(notif.type, notif.data))

        return notifications

    def __extract_notifications_from_log(self, op_code):
        """
        Extract notifications from logs

        :type op_code: str
        :param op_code: String referring to the opcode when notifications should occurred

        :rtype: list
        :return: List of BtGattNotification
        """
        activity_marker = "Activity report"
        gesture_marker = "Gesture detected"
        ignore_msg_marker = "ANCS ignored msgs"
        interrupted_marker = "Interupted notif"
        advertise_marker = "BLE advertise"
        pairing_reset_marker = "BLE pairing reset"
        charge_marker = "Charge event"
        low_battery_marker = "Low battery event"
        crash_marker = "Crash detected"
        switch_off_marker = "Switched off"
        battery_shutdown_marker = "Low battery shutdown"
        thermal_marker = "Thermal shutdown"
        boot_marker = "Boot event"

        self._logger.debug("Getting notifications from clarkTestApp with opcode '{0}'".format(op_code))
        tag_op_code = Base.OPCODE_MARKER + Base.PARAM_SEPARATOR + str(op_code)
        notifications = self.get_android_agent_logger().get_message_triggered_status(tag_op_code)

        # Check if any notification have been found
        if not isinstance(notifications, list):
            return []

        self._logger.debug("Notification found : {0}".format(len(notifications)))
        list_notifs = []
        for notification in notifications:
            filtered_notification = self._extract_results(tag_op_code, notification)
            self._logger.debug("Treat notification {0}".format(filtered_notification))

            if "C_UUID" not in filtered_notification:
                self._logger.debug("No C_UUID attribute => Not a parsable notification")
                continue

            notif = BtGattNotification()
            notif.date = str(filtered_notification["time"])
            notif.service = None
            notif.characteristic = filtered_notification["C_UUID"]
            notif.type = "unknown"
            notif.data = {}
            self._logger.debug("Notification C_UUID={0}".format(notif.characteristic))

            if notif.characteristic == BtServiceClass.GATT_CHARACTERISTIC["GATT_ACTIVITY_VALUE"]:
                notif.service = BtServiceClass.GATT_SERVICE.get('INTEL-EMS', 'UNKNOWN')
                event = filtered_notification["event"]

                # All GATT_ACTIVITY_VALUE notifications have an EventTime data EXCEPT activity notification
                if event != activity_marker:
                    notif.data["EventTime"] = str(filtered_notification["EventTime"])

                if event == activity_marker:
                    notif.type = "activity"
                    notif.data["StartTime"] = str(filtered_notification["StartTime"])
                    notif.data["EndTime"] = str(filtered_notification["EndTime"])
                    notif.data["Activity"] = str(filtered_notification["Activity"])
                    notif.data["StepCount"] = str(filtered_notification["StepCount"])
                elif event == gesture_marker:
                    notif.type = "gesture"
                    notif.data["Gesture"] = str(filtered_notification["Gesture"])
                elif event == ignore_msg_marker:
                    notif.type = "ignore"
                    notif.data["Count"] = str(filtered_notification["Count"])
                elif event == interrupted_marker:
                    notif.type = "interrupted"
                    notif.data["Count"] = str(filtered_notification["Count"])
                elif event == advertise_marker:
                    notif.type = "advertise"
                    notif.data["Duration"] = str(filtered_notification["Duration"])
                elif event == pairing_reset_marker:
                    notif.type = "pairing_reset"
                elif event == charge_marker:
                    notif.type = "charge"
                    notif.data["BatteryLevel"] = str(filtered_notification["BatteryLevel"])
                    notif.data["ChargeStatus"] = str(filtered_notification["ChargeStatus"])
                elif event == low_battery_marker:
                    notif.type = "low_battery"
                elif event == crash_marker:
                    notif.type = "crash"
                    notif.data["PanicType"] = str(filtered_notification["PanicType"])
                elif event == switch_off_marker:
                    notif.type = "switch_off"
                elif event == battery_shutdown_marker:
                    notif.type = "battery_shutdown"
                elif event == thermal_marker:
                    notif.type = "thermal_shutdown"
                    notif.data["Temperature"] = str(filtered_notification["Temperature"])
                elif event == boot_marker:
                    notif.type = "boot"
                    notif.data["BootType"] = str(filtered_notification["BootType"])
                else:
                    # Otherwise ignore it
                    continue
            elif notif.characteristic == BtServiceClass.GATT_CHARACTERISTIC["battery_level"]:
                notif.service = BtServiceClass.GATT_SERVICE.get('battery_service', 'UNKNOWN')
                notif.type = "battery"
                notif.data["value"] = str(filtered_notification["value"])
            elif notif.characteristic == BtServiceClass.GATT_CHARACTERISTIC["SETTINGS_VALUE"]:
                notif.service = BtServiceClass.GATT_SERVICE.get('INTEL-DCS', 'UNKNOWN')
                notif.type = "settings"
                notif.data["value"] = str(filtered_notification["value"])
            else:
                # Otherwise ignore it
                self._logger.debug("Unknown notification")
                continue

            # if notification not yet already present, add it (possible duplicate after log parsing)
            # mean not same date and same characteristic
            for stored_notif in list_notifs:
                if notif.date not in stored_notif.date \
                        and notif.characteristic not in stored_notif.characteristic:
                    list_notifs.insert(0, notif)

        return list_notifs

    def set_bt_power(self, state):
        """
        Sets the Bluetooth power on Companion Phone.

        :type state: str or int
        :param state: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        # pylint: disable=E1101
        if state in ("on", "1", 1, str(BT_STATE.STATE_ON)):
            mode_verbose = str(BT_STATE.STATE_ON)
            state = "ON"
        elif state in ("off", "0", 0, str(BT_STATE.STATE_OFF)):
            mode_verbose = str(BT_STATE.STATE_OFF)
            state = "OFF"
        else:
            self._logger.error("set_bt_power : Parameter mode %s not valid" % state)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid !")

        current_mode = str(self.get_bt_power_status())

        if current_mode == mode_verbose:
            self._logger.info("BT Power already set to {}".format(state))
        else:
            self._logger.info("Settings BT Power to {}".format(state))
            action = "set_bt_state"
            cmd = "-a com.acs.action.{0} -e STATE {1}".format(action, state)
            self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)
            time.sleep(2)
            self._logger.info("BT power set successfully")

    def get_bt_power_status(self):
        """
        Gets the Bluetooth power status.

        :rtype: str
        :return: BT_STATE ('STATE_OFF', 'STATE_ON', 'STATE_TURNING_OFF',
                     'STATE_TURNING_ON')
        """
        action = "get_bt_state"
        cmd = "-a com.acs.action.{0}".format(action)
        result = self._internal_exec(cmd=cmd, timeout=self._uecmd_default_timeout)
        output = result[Base.OUTPUT_MARKER]
        if "on" in str(output).lower():
            return str(BT_STATE.STATE_ON)
        elif "off" in str(output).lower():
            return str(BT_STATE.STATE_OFF)
        else:
            raise DeviceException(DeviceException.FEATURE_NOT_AVAILABLE,
                                  "Unable to retrieve bt state")
