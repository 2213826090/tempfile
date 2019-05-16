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

:organization: SII on behalf of INTEL MCG PSI
:summary: This file implements Local connectivity UECmds
:since: 11 Feb 2014
:author: vgomberx
"""
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE, BluetoothDevice
from acs_test_scripts.Device.UECmd.Interface.LocalConnectivity.ILocalConnectivity import ILocalConnectivity
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE, BT_SCAN_MODE, BT_BOND_STATE, \
    BT_PINVARIANT, BtConState, BtAudioState, BtProfile
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException

import os
import time
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil

from acs_test_scripts.Device.UECmd.UECmdDecorator import need

class BtArgument(object):
    """
    Class enumerating Bt Argument :
    """
    PINCODE = 16
    PASSKEY = 6

class BondState(object):

    """
    Class enumerating Bond state value :
    """
    NAME = "Bond_state"
    BOND_NONE = 10
    BOND_BONDING = 11
    BOND_BONDED = 12


class ScanMode(object):

    """
    Class enumerating Scan mode value :
    """
    NAME = "mode"
    SCAN_MODE_NONE = 20
    SCAN_MODE_CONNECTABLE = 21
    SCAN_MODE_CONNECTABLE_DISCOVERABLE = 23


class BtState(object):

    """
    Class enumerating Bluetooth state value :
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


class LocalConnectivity(Base, ILocalConnectivity):
    """
    Class that handles all local connectivity related operations.
    """
    list_supported_profile = [BtProfile.A2DP, BtProfile.HSP, BtProfile.HID]

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        ILocalConnectivity.__init__(self, device)

    @need('bluetooth', False, str(BT_STATE.STATE_OFF))
    def get_bt_power_status(self):
        """
        Gets the Bluetooth power status.

        :rtype: str
        :return: BT_STATE ('STATE_OFF', 'STATE_ON') There is no 'STATE_TURNING_OFF', 'STATE_TURNING_ON' on windows device
        """
        #function = "GetBTStateByUI"
        function = "QuickGetBTStateByUI"
        module_name, class_name = self._get_module_and_class_names("UIBT")
        #function = "GetBluetoothState"
        #module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")


        args = ''
        output = self._internal_uecmd_exec(module_name, class_name, function, args, 60)

        bluetooth_state = output["bluetooth_state"]

        # Value 0x0 Device is working properly.
        if bluetooth_state == "BT_STATE.STATE_ON":
            bt_power_status = BT_STATE.STATE_ON
        # Value 0x22 Device is disabled.
        elif bluetooth_state == "BT_STATE.STATE_OFF":
            bt_power_status = BT_STATE.STATE_OFF
        elif bluetooth_state == "on":
            bt_power_status = BT_STATE.STATE_ON
        elif bluetooth_state == "off":
            bt_power_status = BT_STATE.STATE_OFF
        # Other Windows Configuration Manager error code
        else:
            bt_power_status = BT_STATE.STATE_OFF
            # see more information here (http://msdn.microsoft.com/en-us/library/aa394216%28v=vs.85%29.aspx)
            self._logger.error("GetBluetoothState return an unknown error code <%s>", bt_power_status)

        return str(bt_power_status)

    @need('bluetooth', False)
    def set_bt_power(self, mode):
        """
        Sets the Bluetooth power.

        :type mode: str or int
        :param mode: can be ('on', '1', 1, STATE_ON) to enable
                            ('off', '0', 0, STATE_OFF) to disable

        :return: None
        """
        # function = "TurnOnOffBTByUI"
        function = "QuickTurnOnOffBTByUI"
        # function = "TurnOnOffBluetoothFromNetworkAdapter"
        # Get the method and class name of the UEcommand on the embedded side
        args = "action="
        module_name, class_name = self._get_module_and_class_names("UIBT")
        # module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        if mode == 'on' or mode == "1" or mode == 1 or mode == 'STATE_ON':
            args += "TurnOn"
        elif mode == 'off' or mode == "0" or mode == 0 or mode == 'STATE_OFF':
            args += "TurnOff"
        else:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "<mode> in set_bt_power(%s)", mode)

        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, function, args, 120)

    @need('bluetooth')
    def bluetooth_menu_settings(self):
        """
        Enter Bluetooth menu setting or exit from Bluetooth menu settings
        """
        self._logger.debug("[NOT IMPLEMENTED] bluetooth_menu_settings not relevant on Windows")

    @need('bluetooth')
    def set_bt_discoverable(self, mode, timeout):
        """
        Sets the device discoverable.

        :type mode: str or int
        :param mode: only none and both are used for windows
          can be ('noscan', 'none', SCAN_MODE_NONE) for no scan
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
        # Always visible doesn't exist on Windows device
        # it's because we set the timeout at 5 hours
        if timeout == 0 and mode == 'both':
            timeout = 5 * 3600
        method_name = "BluetoothEnableDiscovery"
        # Get the method and class name of the UEcommand on the embedded side
        args = "timeout=" + str(timeout)
        args += " mode=" + str(mode)
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        start_delay = 0
        repeat_delay = 0
        total_duration = timeout
        # We use schedule_uecmd because enable discovery it's a blocking function on windows
        self._logger.info("Launch Async Uecmd : " + method_name + " " + args)
        self.schedule_uecmd(module_name, class_name, method_name, args, start_delay, repeat_delay, total_duration)
        self._logger.info("Set BT discoverable - success")

    @need('bluetooth')
    def set_bt_name(self, name):
        """
        Sets the device Bluetooth name.

        @type name: string
        @param name: name of the Blue tooth device
        @return: None
        """
        method_name = "BluetoothSetName"
        # Get the method and class name of the UEcommand on the embedded side
        args = "bluetooth_name=" + name
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")

        # Launch the UEcmd on the embedded side
        bt_ame_successfully_change = self._internal_uecmd_exec(module_name, class_name, method_name, args, 120)
        if bt_ame_successfully_change:
            self._logger.info("Set BT Name - success")
        else:
            self._logger.error("Set BT Name - fail")

    @need('bluetooth')
    def get_bt_adapter_address(self):
        """
        Gets Bluetooth address.

        :rtype: str
        :return: Bluetooth address
        """
        method_name = "getBluetoothAddress"
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        args = ''

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, method_name, args, 60)
        address = output["bluetooth_address"]

        return address

    @need('bluetooth')
    def bt_scan_devices(self):
        """
        Scans remote Bluetooth devices.

        :rtype: list of BluetoothDevice object
        :return: list of founded adapters (name/address)
        """
        method_name = "btScanDevices"
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        args = ''
        device_list = []

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, method_name, args, 60)
        output_device_list = output["device_list"]

        for element in output_device_list:
            bt_dev = BluetoothDevice()
            if "deviceAddress" in element:
                device_address_info = element["deviceAddress"]
                temp_bt_dev_address = device_address_info["dataString"]
                # add 2 dot in bt address
                if len(temp_bt_dev_address) == 12:
                    temp_bt_dev_address = temp_bt_dev_address[:2] + ':' \
                                          + temp_bt_dev_address[2:4] + ':' \
                                          + temp_bt_dev_address[4:6] + ':' \
                                          + temp_bt_dev_address[6:8] + ':' \
                                          + temp_bt_dev_address[8:10] + ':' \
                                          + temp_bt_dev_address[10:]
                bt_dev.address = temp_bt_dev_address
            if "deviceName" in element:
                bt_dev.name = element["deviceName"]
            device_list.append(bt_dev)

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

    @need('nfc', False, "Device does not have nfc capability")
    def nfc_enable(self):
        """
        Enable NFC interface

        :return: list of founded adapters (interface/address)
        """
        self._logger.debug("[NOT IMPLEMENTED] nfc_enable on Windows")

    @need('nfc', False, "Device does not have nfc capability")
    def nfc_disable(self):
        """
        Disable NFC interface

        :return: None
        """
        self._logger.debug("[NOT IMPLEMENTED] nfc_disable on Windows")

    @need('nfc', False, "OFF")
    def get_nfc_status(self):
        """
        Retreive the NFC Status

        :rtype: str
        :return: Return the NFC status "OFF", "TURNING_ON", "ON", "TURNING_OFF", "UNKNOWN"
        """
        self._logger.debug("[NOT IMPLEMENTED] get_nfc_status on Windows")
        return "OFF"

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
        if len(str(pincode)) > BtArgument.PINCODE:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "pincode \'%s\' is too long (16 char Max)" \
                                     % pincode)

    def _validate_passkey_arg(self, passkey):
        """
        Validate passkey argument
        """
        if (len(str(passkey)) > BtArgument.PASSKEY) or (not str(passkey).isdigit()):
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

    def _unpair_first_if_needed(self, remote_device_addr, reconnect):
        """
        Unpair if needed
        """
        if reconnect in ("on", "1", 1):  # if device is already paired, then force unpair
            pairedlist = self.list_paired_device()
            for element in pairedlist:
                if str(element.address).upper() == remote_device_addr:
                    self.unpair_bt_device(remote_device_addr)
        elif not reconnect in ("off", "0", 0):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "reconnect : Bad parameter value %s" % (str(reconnect)))

    @need('bluetooth')
    def unpair_bt_device(self, remote_device_addr):
        """
        Remove the remote Bluetooth device.

        :type remote_device_addr: str
        :param remote_device_addr: remote Bluetooth device address

        :return: None
        """
        self._logger.info("Unpairing from device %s" % remote_device_addr)

        # method = "unpairDevice"
        method = "unpairDeviceWithUI"
        # Get the method and class name of the UEcommand on the embedded side
        args = "Address=" + str(remote_device_addr).upper()
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")

        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, method, args, 120)

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
            (pinRes is pin variant 'PIN' or 'PASSKEY' or 'REPLY' or 'NONE')
        """
        self._logger.info("Pairing to device : %s" %
                          remote_device_addr)

        self._validate_pairing_args(remote_device_addr, replyval, pincode, passkey)
        self._unpair_first_if_needed(remote_device_addr, reconnect)

        method = "pairDevice"
        # Get the method and class name of the UEcommand on the embedded side
        args = "Address=" + str(remote_device_addr).upper()
        args += " PairRep=" + str(replyval)
        args += " Pincode=" + str(pincode)
        args += " Passkey=" + str(passkey)
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, method, args, 120)

        result = output["bond_state"]
        # No pin variant verification on windows
        pinres = BT_PINVARIANT.NONE

        if str(result).lower() == "none":
            return BT_BOND_STATE.BOND_NONE, pinres
        elif str(result).lower() == "bonded":
            return BT_BOND_STATE.BOND_BONDED, pinres
        else:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                  "pairDevice: Bad return value %s" % str(result))

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
        args = ""
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, method, args, 60)
        listPairedDevice = output["listPairedDevice"]

        for element in listPairedDevice:
            if "address" in element:
                btdev = BluetoothDevice()
                btdev.address = element["address"]
                device_list.append(btdev)
            if "name" in element:
                btdev = BluetoothDevice()
                btdev.name = element["name"]
                device_list.append(btdev)

        return device_list

    @need('bluetooth')
    def connect_bt_device(self, remote_device_addr, profile):
        """
        Connects to a remote Bluetooth device profile.

        :type remote_device_addr: str
        :param remote_device_addr: address of the remote Bluetooth device

        :type profile: str
        :param profile: rofile to connect with like hsp, a2dp, nap...

        :rtype: boolean
        :return: connection status (true for connection succeeded)
        """
        self._logger.info("Trying to connect to Bluetooth %s profile" % profile)
        method = "connectProfile"
        profile = str(profile).upper()
        if profile not in self.list_supported_profile:
            self._logger.error("not supported profile '%s'" % profile)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s: not supported profile %s" % (method, profile))

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        args = "Address=" + str(remote_device_addr).upper()
        args += " Profile=" + str(profile)
        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, method, args, 60)
        result = output["connection_state"]
        # Check correct profile has been connected successfully
        if str(BtProfile.NAME).lower() in result:
            if result[str(BtProfile.NAME).lower()] != profile:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "%s: Bad profile returned %s" % (method, result[str(BtProfile.NAME).lower()]))
        else:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "%s: No profile returned" % method)

        if str(BtConState.NAME).lower() in result:
            # Check connection succeeded
            if result[str(BtConState.NAME).lower()] == BtConState.CONNECTED:
                return True
            elif result[str(BtConState.NAME).lower()] == BtConState.DISCONNECTED:
                return False
            else:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "%s: Bad connection return value %s" % (method, result[str(BtConState.NAME).lower()]))
        else:
            self._logger.info("Connection State not found")
            if "output" in result:
                msg = "%s ERROR: %s" % (method, result["output"])
            else:
                msg = "%s ERROR" % method
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

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
        if profile not in self.list_supported_profile:
            self._logger.error("not supported profile '%s'" % profile)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s: not supported profile %s" % (method, profile))

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        args = "Address=" + str(remote_device_addr).upper()
        args += " Profile=" + str(profile)
        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, method, args, 120)
        result = output["connection_state"]
        # Check correct profile has been disconnected successfully
        if str(BtProfile.NAME).lower() in result:
            if result[str(BtProfile.NAME).lower()] != profile:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "%s: Bad profile returned %s" % (method, result[BtProfile.NAME]))
        else:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                  "%s: No profile returned" % method)

        if str(BtConState.NAME).lower() in str(result).lower():
            # Check disconnection succeeded
            if result[str(BtConState.NAME).lower()] == BtConState.DISCONNECTED:
                disc_stat = True
            elif result[str(BtConState.NAME).lower()] == BtConState.CONNECTED:
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

        if profile not in self.list_supported_profile:
            self._logger.error("not supported profile '%s'" % profile)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s: not supported profile %s" % (method, profile))

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        args = "Address=" + str(remote_device_addr).upper()
        args += " Profile=" + str(profile)
        # Launch the UEcmd on the embedded side
        result = self._internal_uecmd_exec(module_name, class_name, method, args, 60)
        output = result["connection_state"]
        if str(BtConState.NAME).lower() in output:
            self._logger.info("Bluetooth profile %s is %s" % (profile, output[str(BtConState.NAME).lower()]))
            # Check connection succeeded
            if output[str(BtConState.NAME).lower()] not in BtConState.d:
                msg = "%s: Bad connection return value %s" % (method, output[str(BtConState.NAME).lower()])
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
            elif output[str(BtConState.NAME).lower()] == BtConState.UNKNOWN:
                msg = "unknown connection state"
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

            con_stat_res = BtConState.d[output[str(BtConState.NAME).lower()]]

        else:
            self._logger.info("Connection State not found")
            if "output" in output:
                msg = "%s ERROR: %s" % (method, output["output"])
            else:
                msg = "%s ERROR" % method
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

        return con_stat_res

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
        args = "Address=" + str(remote_device_addr).upper()
        args += " PairRep=" + str(replyval)
        args += " Pincode=" + str(pincode)
        args += " Passkey=" + str(passkey)
        args += " Timeout=" + str(timeout)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")

        start_delay = 0
        repeat_delay = 0
        total_duration = timeout
        self._logger.info("Launch Async Uecmd : " + method + " " + args)
        # We use schedule_uecmd because wait pair request it's a blocking function on windows
        self.schedule_uecmd(module_name, class_name, method, args, start_delay, repeat_delay, total_duration)
        # We wait between 2 uecmd and let the time at windows to prepare to pair
        time.sleep(10)
        # We check the status of pairing request
        method = "getStatusWaitPairReq"
        args = "Address=" + str(remote_device_addr).upper()
        output = self._internal_uecmd_exec(module_name, class_name, method, args, timeout)
        result = output["bond_state"]

        if str(result).lower() == "bonding":
            return BT_BOND_STATE.BOND_BONDING
        elif str(result).lower() == "bonded":
            return BT_BOND_STATE.BOND_BONDED
        elif str(result).lower() == "none":
            return BT_BOND_STATE.BOND_NONE
        else:
            msg = "waitPairReq ERROR: %s" % str(result).lower()
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

    @staticmethod
    def recover_list_files_names_2_send(list2send):
        """
        recover the list of files 2 send without the path
        :type list2send: list
        :param list2send: the current list file name
        :return: str with the list of files name
        """
        list_files_name_2_send = []
        for element in list2send:
            elt_split = element.split('/')
            list_files_name_2_send.append(elt_split[-1])
        string2send = ""
        for element in list_files_name_2_send:
            string2send += "\\\"" + element + "\\\"" + " "
        return string2send

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

        self._logger.info("Send the file")
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        start_delay = 0
        repeat_delay = 0
        total_duration = 120
        list_filename = filename.split(",")
        if list_filename.__len__() == 1:
            method = "sendOPPfileWithUI"
            filePath = list_filename[0].replace("/", "\\")
            args = "destination_address=" + str(destination_address).upper()
            args += " filePath=" + str(filePath)
            self._logger.info("Launch Async Uecmd : " + method + " " + args)
            # self.schedule_uecmd(module_name, class_name, method, args, start_delay, repeat_delay, total_duration)
            self._internal_uecmd_exec(module_name, class_name, method, args, 120)
        else:
            list_of_files_2_send = LocalConnectivity.recover_list_files_names_2_send(list_filename)
            method = "sendOPPmultiplefilesWithUI"
            args = "destination_address=" + str(destination_address).upper()
            args += " list_of_files=" + "\"" + str(list_of_files_2_send) + "\""
            args += " folder_path="
            self._logger.info("Launch Async Uecmd : " + method + " " + args)
            self._internal_uecmd_exec(module_name, class_name, method, args, 120)

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
        self._logger.info("BT Opp get files checksum")
        if not folder_name:
            folder_name = self.get_default_directory()

        result = {}
        module_name, class_name = self._get_module_and_class_names("DeviceSystem")
        method = "GetFileChecksum"
        for file_name in file_list:
            args = "filename=" + folder_name+'\\'+file_name
            output = self._internal_uecmd_exec(module_name, class_name, method, args, 60)
            response = str(output["fileChecksum"])
            if not response or ("No such file or directory" in response):
                msg = "File %s not found" % file_name
                self._logger.error(msg)
                raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

            # response is in the form "<checksum>"
            result[file_name] = response

        return result

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
        self._logger.info("BT Check Opp service")
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        method = "receiveOPPfile"
        args = ""
        output = self._internal_uecmd_exec(module_name, class_name, method, args, 60)
        raw_ssids = output["list_dict_receive_file_prop"]
        return raw_ssids

    @need('bluetooth')
    def bt_opp_clean_notification_list(self):
        """
        Clean (Empty) the BT OPP notification list
        """
        self._logger.info("BT Clean Opp service list")
        self._logger.info("emptyOPPNotificationsList not exist on Windows")

    @need('bluetooth')
    def bt_opp_init(self, file_to_be_received):
        """
        If already in filesystem, remove the file to be received.
        :type file_to_be_received: str
        :param file_to_be_received: filename of the file to be received soon
        :return: None
        """
        folder_name = self.get_default_directory()
        module_name, class_name = self._get_module_and_class_names("DeviceSystem")
        method = "DeleteFile"
        args = "file2delete=" + folder_name+'\\'+file_to_be_received
        self._internal_uecmd_exec(module_name, class_name, method, args, 60)

    @need('bluetooth')
    def bt_start_opp_server(self):
        """
        Start Opp server for windows
        """
        self._logger.info("Start Opp Server for Windows")
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        method = "startOppServer"
        args = ""
        self._internal_uecmd_exec(module_name, class_name, method, args, 120)

    @need('bluetooth')
    def bt_stop_opp_server(self):
        """
        Stop Opp server for windows
        """
        self._logger.info("Stop Opp Server for Windows")
        module_name, class_name = self._get_module_and_class_names("BluetoothConnectivity")
        method = "stopOppServer"
        args = ""
        self._internal_uecmd_exec(module_name, class_name, method, args, 120)

    def get_default_directory(self):
        # Get the username
        module_name, class_name = self._get_module_and_class_names("DeviceSystem")
        method = "getUserName"
        args = ""
        output = self._internal_uecmd_exec(module_name, class_name, method, args, 60)
        username = output["username"]
        # Default folder for bluetooth files
        if username:
            folder_users = "C:\Users"
            folder_documents = "Documents"
            temp_path = os.path.join(folder_users, username)
            folder_name = os.path.join(temp_path, folder_documents)
        else:
            folder_name = "C:\Users\Public\Documents"
        return str(folder_name)
