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

from acs_test_scripts.Device.UECmd.Interface.LocalConnectivity.ILocalConnectivity import ILocalConnectivity
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Device.UECmd.UECmdTypes import BluetoothDevice

from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE, BT_BOND_STATE, BtConState
import time

import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from twisted.spread.flavors import remote_atom

from acs_test_scripts.Utilities.CommandServerUtilities import CommandServerApi, SocketProtocol

class LocalConnectivity(Base, ILocalConnectivity, CommandServerApi):
    """
    Class that handles all local connectivity related operations.
    """

    # Pairing replies
    _PAIRING_REPLIES = ["reject", "accept", "timeout"]

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        ILocalConnectivity.__init__(self, device)
        CommandServerApi.__init__(self, SocketProtocol(device.get_device_ip()))
        self.__path_env = "PATH=/usr/sbin/:/usr/bin:/usr/local/bin:/bin"
        self.__bcm_bt_ctrl_on = "rfkill unblock bluetooth"
        self.__bcm_bt_ctrl_off = "rfkill block bluetooth"
        self.__hci0 = "rfkill list"

    @need('bluetooth')
    def bt_reset_device(self):
        """
        Resets a Bluetooth adapter. Turn OFF then ON and check power state is successfully ON
        :return: None
        """
        self._logger.debug('bt_reset_device not used')

    @need('bluetooth', False)
    def set_bt_power(self, mode):
        """
        Sets the bluetooth power to off or on.
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        if mode in ["on", "1", 1, str(BT_STATE.STATE_ON)]:
            self._start_bt()
        elif mode in ["off", "0", 0, str(BT_STATE.STATE_OFF)]:
            self._stop_bt()
        else:
            msg = "Wrong MODE given"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    @need('bluetooth', False, str(BT_STATE.STATE_OFF))
    def get_bt_power_status(self):
        """
        Gets the Bluetooth power status.
        :rtype: str
        :return: BT_STATE ('STATE_OFF', 'STATE_ON', 'STATE_TURNING_OFF',
                     'STATE_TURNING_ON')
        """
        cmd = "%s %s" %(self.__path_env, self.__hci0)

        status, output = self._internal_exec(cmd)
        if status == True and "hci0" in output:
            self._logger.info("BT hci0 interface is up, BT_STATE_ON")
            result = str(BT_STATE.STATE_ON)
        else:
            self._logger.info("BT hci0 interface not found!, BT_STATE_OFF")
            result = str(BT_STATE.STATE_OFF)

        return str(result)

    def _check_bt_power_state(self, should_be_on, timeout=10):
        """
        Check BT power state
        :type should_be_on: bool
        :param should_be_on: flag that indicates if BT power should be on
        :type timeout: int
        :param timeout: maximum time to wait for expected BT power state
        :rtype: bool
        :return: True if expected BT power state is reached before timeout
                 False otherwise
        """
        cmd = "%s %s" %(self.__path_env, self.__hci0)

        begin_time = time.time()
        while time.time() - begin_time < timeout:
            status, output = self._internal_exec(cmd)
            if not status:
                self._logger.error("{0} command failed".format(self.__hci0))
            elif "hci0" in output and should_be_on:
                self._logger.info("BT hci0 interface is up")
                break
            elif "hci0" not in output and not should_be_on:
                self._logger.info("BT hci0 interface is down")
                break
            time.sleep(1)
        else:
            msg = "Check BT hci0 interface power state timeout!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    def _start_bt(self):
        """
        Start bcm_bluetooth_control on DUT.
        :return:
            None
        """
        self._logger.debug("Starting bcm_bluetooth_control")

        cmd = "%s %s" %(self.__path_env, self.__bcm_bt_ctrl_on)
        status, output = self._internal_exec(cmd)

        if status == False:
            msg = "Failed to power ON bluetooth"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        # Check BT hci0 is up
        self._check_bt_power_state(True)

    def _stop_bt(self):
        """
        Stop bcm_bluetooth_control on DUT.
        :rtype: tuple
        :return: Output status and output log
        """
        self._logger.debug("Stopping bcm_bluetooth_control")

        # turn BT RF off
        cmd = "%s %s" % (self.__path_env, self.__bcm_bt_ctrl_off)
        status, output = self._internal_exec(cmd)

        if status == False:
            msg = "Failed to power OFF bluetooth"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        # Check BT hci0 is up
        self._check_bt_power_state(False)

    def nfc_disable(self):
        """
        Disables NFC interface
        :return: None
        """
        self._logger.debug("nfc_disable: Not used on Linux devices")

    def nfc_enable(self):
        """
        Enables NFC interface
        :return: None
        """
        self._logger.debug("nfc_enable: Not used on Linux devices")

    def get_nfc_status(self):
        """
        Retrieve the NFC Status
        :rtype: str
        :return: Return the NFC status "OFF", "TURNING_ON", "ON", "TURNING_OFF", "UNKNOWN"
        """
        self._logger.debug("get_nfc_status: Not used on Linux devices")
        return "OFF"

    def bt_scan_devices(self):
        """
        Scans remote bluetooth devices.
        Looks for all visible bluetooth devices.
        :rtype: list of BluetoothDevice object
        :return: list of found devices (name/address)
        """
        self._logger.info("Scanning for bluetooth devices")

        args = {"timeout": self._device.get_config("btDefaultScanTimeout", 10, int)}
        status, result = self.send_cmd("scan", args)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        status, result = self.get_result(result)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        res = []
        for mac_addr in result:
            btdev = BluetoothDevice()
            btdev.name = result[mac_addr]
            btdev.address = mac_addr
            res.append(btdev)

        return res

    def bt_find_device(self, remote_device_info):
        """
        Looks for a specific bluetooth device.
        :type remote_device_info: str
        :param remote_device_info: address of the device to find
        :rtype: boolean
        :return: True if found , False otherwise
        """
        self._logger.info("Scanning for %s bluetooth device" % (remote_device_info))

        args = {
            "addr": remote_device_info,
            "timeout": self._device.get_config("btDefaultFindDeviceTimeout", 30, int)
        }
        status, result = self.send_cmd("find_device", args)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        status, result = self.get_result(result)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            self._logger.error(result)
            return False

        self._logger.info("Device %s found" % remote_device_info)
        return True

    def set_bt_prop(self, name, value):
        """
        Set bluetooth controller property
        :type name: str
        :param name: property name to set
        :type value: object
        :param value: property value to set
        :rtype: boolean
        :return:
            - True if set succeeded
            - False in case of failure
        """
        self._logger.debug("Set bluetooth controller property %s to %s" % (name, str(value)))

        args = {"name": name, "value": value}
        status, result = self.send_cmd("setprop", args)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            self._logger.error(result)
            return False

        return True

    def get_bt_prop(self, name):
        """
        Get bluetooth controller property value
        :type name: str
        :param name: property to get
        :rtype: object
        :return: property value
        """
        self._logger.debug("Get bluetooth controller %s property value" % (name))

        status, result = self.send_cmd("getprop", {"name": name})

        if status == CommandServerApi.SRV_CMD_FAILURE:
            self._logger.error(result)

        self._logger.debug("Bluetooth controller property %s is %s" % (name, str(result)))
        return result

    def set_bt_pairable(self, mode):
        """
        Sets bluetooth device pairable status
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        self._logger.info("Set bluetooth pairable %s" % ("ON" if mode in ['on', '1', 1] else "OFF"))
        pairable = True
        if mode in ['off', '0', 0]:
            pairable = False
        self.set_bt_prop("Pairable", pairable)

    def get_bt_pairable_status(self):
        """
        Returns the bluetooth device pairable status.
        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        self._logger.info("Get bluetooth pairable status")
        pairable = self.get_bt_prop("Pairable")

        status = 0
        if pairable:
            status = 1

        return status

    def set_bt_name(self, name):
        """
        Sets the device bluetooth name.
        :type name: str
        :param name: bluetooth device name to set
        :return: None
        """
        self._logger.info("Set bluetooth device name to %s" % (name))
        self.set_bt_prop("Alias", name)

    def set_bt_discoverable(self, mode, timeout=0):
        """
        Sets bluetooth device discoverable mode and timeout
        :type mode: str
        :param mode:
            - "on"  => discoverable
            - "off" => not discoverable
        :type timeout: int
        :param timeout: can be 0 for never, or any positive integer value
        :return: None
        """
        if mode not in ["on", "off"]:
            self._logger.error("Parameter mode %s is not valid" % str(mode))
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                "Parameter mode is not valid!"
            )

        if timeout < 0:
            self._logger.error(
                "Parameter timeout %d is not valid => use default value (0)" % (timeout)
            )
            timeout = 0

        discoverable = True
        if mode == "off":
            discoverable = False

        if timeout == 0:
            self._logger.info("Set bluetooth discoverable timeout to never")
        else:
            self._logger.info("Set bluetooth discoverable timeout to %d seconds" % (timeout))
        self.set_bt_prop("DiscoverableTimeout", timeout)

        self._logger.info("Set bluetooth device discoverable %s" % (mode))
        self.set_bt_prop("Discoverable", discoverable)

    def get_bt_adapter_address(self):
        """
        Get bluetooth controller address
        :rtype: str
        :return: bluetooth controller address
        """
        self._logger.info("Get bluetooth controller address")
        return self.get_bt_prop("Address")

    def list_paired_device(self):
        """
        Lists all paired devices
        :rtype: list of BluetoothDevice object
        :return: list of paired devices (name/address)
        """
        self._logger.info("Retrieving list of paired devices")

        status, result = self.send_cmd("paired_devices")

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        devices = []
        for mac_addr in result:
            btdev = BluetoothDevice()
            btdev.name = result[mac_addr]
            btdev.address = mac_addr
            devices.append(btdev)

        return devices

    def unpair_bt_device(self, remote_device_addr):
        """
        Remove the remote bluetooth paired device.
        :type remote_device_addr: str
        :param remote_device_addr: remote bluetooth device address
        :return: None
        """
        self._logger.info("Unpairing device %s" % remote_device_addr)

        status, result = self.send_cmd("remove", {"addr": remote_device_addr})

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        self._logger.info(result)

    def pair_to_device(self, remote_device_addr, reconnect="off",
                       replyval=1, pincode="0000", passkey=0):
        """
        Requests pairing to a bluetooth remote device.
        :type remote_device_addr: str
        :param remote_device_addr: remote bluetooth device address
                                   format 00:00:00:00:00:00
        :type reconnect: str or int
        :param reconnect: can be ('on', '1', 1) to enable
                                 ('off', '0', 0) to disable
        :type replyval: int
        :param replyval: pairing request reply value
                         0 = negative reply (reject or cancel)
                         1 = positive reply (accept)
                         2 = timeout (wait to trigger timeout in device to pair with)
        :type pincode: str
        :param pincode: bluetooth device PIN code for pairing process
                        (16 characters Max)
        :type passkey: int
        :param passkey: bluetooth device passkey for pairing process
                       (6 digits Max)
        :rtype: list of 2 strings
        :return: (bond_state 'BOND_BONDED' or 'BOND_NONE') and
            (pinres is pin variant 'PIN' or 'PASSKEY' or 'REPLY' or 'NONE')
        """
        self._logger.info("Pairing to device: %s" % remote_device_addr)

        # Get pairing reply
        if replyval not in range(len(self._PAIRING_REPLIES)):
            msg = "Invalid replyval parameter: %d" % (replyval)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        args = {
            "passkey": passkey,
            "pin_code": pincode,
            "reply": self._PAIRING_REPLIES[replyval],
            "addr": remote_device_addr
        }
        status, result = self.send_cmd("pair", args)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        status, result = self.get_result(result)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            self._logger.info(result)
            return BT_BOND_STATE.BOND_NONE, "NONE"

        return BT_BOND_STATE.BOND_BONDED, "NONE"

    def wait_for_disconnection(self, remote_device_addr):
        """
        Wait for bluetooth device disconnection
        :type remote_device_addr: str
        :param remote_device_addr: bluetooth device address to wait disconnection for
        """
        self._logger.info("Wait for %s device disconnection" % (remote_device_addr))

        args = {
            "addr": remote_device_addr,
            "timeout": self._device.get_config("btDefaultDisconnectionTimeout", 10, int)
        }
        status, result = self.send_cmd("wait_for_disconnection", args)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        status, result = self.get_result(result)

        if status == CommandServerApi.SRV_CMD_SUCCESS:
            self._logger.info(result)
        else:
            self._logger.error(result)

    def wait_for_connection(self, remote_device_addr, profile=None):
        """
        Wait for bluetooth device connection
        :type remote_device_addr: str
        :param remote_device_addr:  bluetooth remote device to wait connection for
        :type profile: str
        :param profile: bluetooth profile to handle for device connection
        :return: None
        """
        self._logger.info("Wait for connection from %s device" % (remote_device_addr))

        args = {
            "addr": remote_device_addr,
            "profile": profile, "timeout": self._device.get_config("btDefaultConnectionTimeout", 10, int)
        }
        status, result = self.send_cmd("wait_for_connection", args)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        status, result = self.get_result(result)

        if status == CommandServerApi.SRV_CMD_SUCCESS:
            self._logger.info(result)
        else:
            self._logger.error(result)

    def wait_for_pairing(self, remote_device_addr=None, reconnect=0, replyval=1, pincode="0000",
                         passkey=0, timeout=20):
        """
        Wait for a pairing request from a bluetooth remote device.
        :type remote_device_addr: str
        :param remote_device_addr: remote bluetooth device address
                                   format 00:00:00:00:00:00
                                   In case of None address, device will wait for the first pairing
                                   request
        :type replyval: int
        :param replyval: pairing request reply value
                         0 = Negative reply
                         1 = Positive reply
                         2 = trigger timeout in device to pair with
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
        if remote_device_addr is not None:
            self._logger.info("Wait for pairing to device: %s" % remote_device_addr)
        else:
            self._logger.info("Wait for first pairing request")

        # Get pairing reply
        if replyval not in range(len(self._PAIRING_REPLIES)):
            msg = "Invalid pairing reply value %d" % (replyval)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        args = {
            "passkey": passkey,
            "pin_code": pincode,
            "timeout": timeout,
            "reply": self._PAIRING_REPLIES[replyval],
            "addr": remote_device_addr
        }

        self._logger.debug("UECMD args: %s" % args)

        status, result = self.send_cmd("wait_pairing", args)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        status, result = self.get_result(result)

        if status == CommandServerApi.SRV_CMD_FAILURE:
            self._logger.info(result)
            return BT_BOND_STATE.BOND_NONE

        return BT_BOND_STATE.BOND_BONDING

    def get_bt_pairable_timeout(self):
        """
        Returns the timeout for the pairable  mode.
        :rtype: int
        :return: pairable timeout value
        """
        self._logger.info("Get pairable timeout")
        return self.get_bt_prop("PairableTimeout")

    def get_bt_discoverable_timeout(self):
        """
        Returns the timeout for the discoverable mode.
        :rtype: int
        :return: discoverable timeout value
        """
        self._logger.info("Get discoverable timeout")
        return self.get_bt_prop("DiscoverableTimeout")

    def connect_bt_device(self, remote_device_addr, profile):
        """
        Connects to a remote bluetooth device profile.
        :type remote_device_addr: str
        :param remote_device_addr: bluetooth remote device address
        :type profile: str
        :param profile: profile to connect to (hsp, a2dp, nap...)
        :rtype: boolean
        :return: connection status (true for connection succeeded)
        """
        self._logger.info("Trying to connect to bluetooth %s profile" % profile)

        args = {"addr": remote_device_addr, "profile": profile}

        status, result = self.send_cmd("connect", args)
        status, result = self.get_result(result)

        connected = False
        if status == CommandServerApi.SRV_CMD_SUCCESS:
            self._logger.info("Connected to %s device" % (remote_device_addr))
            connected = True
        else:
            self._logger.error("Connection to %s device failed" % (remote_device_addr))

        return connected

    def disconnect_bt_device(self, remote_device_addr, profile):
        """
        Disconnects from a remote bluetooth device profile.
        :type remote_device_addr: str
        :param remote_device_addr: the bluetooth remote device address
        :type profile: str
        :param profile: profile to disconnect from (hsp, a2dp, pan...)
        :rtype: boolean
        :return: disconnection status (true for disconnection succeeded)
        """
        self._logger.info("Trying to disconnect from bluetooth %s profile" % profile)

        args = {"addr": remote_device_addr, "profile": profile}
        status, result = self.send_cmd("disconnect", args)
        status, result = self.get_result(result)

        disconnected = False
        if status == CommandServerApi.SRV_CMD_SUCCESS:
            self._logger.info("Disconnected from %s device" % (remote_device_addr))
            disconnected = True
        if status == CommandServerApi.SRV_CMD_FAILURE:
            self._logger.error("Disconnection from %s device failed" % (remote_device_addr))

        return disconnected

    def get_bt_connection_state(self, remote_device_addr, profile=None):
        """
        Get the bluetooth connection state to a specific profile.
        :type profile: str
        :param profile: profile to check connection for (hsp, a2dp...)
                        If profile is None, it will check for at least one connected profile
        :type remote_device_addr: str
        :param remote_device_addr: remote bluetooth device address
        :rtype: int
        :return: the connection state
                - BtConState.d[BtConState.DISCONNECTED] for disconnected
                - BtConState.d[BtConState.CONNECTED] for connected
                - BtConState.d[BtConState.CONNECTING] for connecting
                - BtConState.d[BtConState.DISCONNECTING] for disconnecting
        """
        msg = "Get connection state with %s bluetooth device" % (remote_device_addr)
        if profile is not None:
            msg += " %s profile" % (profile)
        self._logger.info(msg)

        args = {"addr": remote_device_addr, "name": "Connected"}
        status, result = self.send_cmd("device_getprop", args)

        connected = False
        if status == CommandServerApi.SRV_CMD_SUCCESS:
            connected = result
        elif status == CommandServerApi.SRV_CMD_FAILURE:
            self._logger.error("Failed to get connection status with %s device" % remote_device_addr)

        return connected

    def set_bt_adapter_power(self, mode):
        """
        Sets the bluetooth adapter power mode.
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        self._logger.info("Set bluetooth adapter power to %s" % str(mode))

        powered = None
        if mode in ['on', '1', 1]:
            powered = True
        elif mode in ['off', '0', 0]:
            powered = False
        else:
            msg = "Wrong mode given"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        self.set_bt_prop("Powered", powered)

    def set_ssp_mode(self, mode="on"):
        """
        Sets Simple Secure Pairing mode to enable/disable it and force
        device to use Legacy Pairing mode instead.
        :type iface: str
        :param iface: bluetooth interface (hci0, hci1, etc.)
        :type mode: str
        :param mode: SSP mode: "off" => disabled, "on" => enabled
        :return None
        """
        cmd = "hciconfig " + self._device.get_config("btIface", "hci0") + " sspmode "

        if mode == "off":
            self._logger.info("Disable bluetooth SSP mode")
            cmd += "0"
        elif mode == "on":
            self._logger.info("Enable bluetooth SSP mode")
            cmd += "1"
        else:
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                "Mode <%s> is not valid!" % str(mode)
            )

        status, output = self._internal_exec(cmd, 3)

    def enable_service(self, enable, service):
        """
        Enables/disables bluetooth service
        :type enable: boolean
        :param enable: enable service if True, disable it otherwise
        :type service: str
        :param service: bluetooth service to enable/disable
        :return None
        """
        if service not in ["nap", "gn", "panu"]:
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                "Service <%s> is not valid!" % str(service))

        pid = None
        if service == "nap":
            if enable:
                cmd = "start_nap"
                self.set_bt_tethering_power('on')
            else:
                self.set_bt_tethering_power('off')
                cmd = "stop_nap"

        status, result = self.send_cmd(cmd, {})

        if status == CommandServerApi.SRV_CMD_FAILURE:
            msg = "Failed to start {0} service".format(service)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def trust_device(self, remote_device_addr, trust):
        """
        Trust/Untrust paired device
        :type remote_device_addr: str
        :param remote_device_addr: remote bluetooth address of device to trust
        :type trust: boolean
        :param trust:
            - True to trust remote devie
            - False to untrust remote device
        :return: None
        """
        msg = "%s %s remote device" % ("Trust" if trust else "Untrust", remote_device_addr)
        self._logger.info(msg)

        args = {"addr": remote_device_addr, "name": "Trusted", "value": trust}
        self.send_cmd("device_setprop", args)

    def set_bt_tethering_power(self, state):
        """
        Enable / Disable the Bluetooth Tethering feature.
        :type state: str or int
        :param state: can be 'on' to enable
                             'off' to disable
        """
        state = str(state).lower()
        self._logger.info("Set BT Tethering to %s" % state)

        if state not in ['on', 'off']:
            msg = "set_bt_tethering_power : " + \
                "Parameter state [%s] not valid" % state
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        args = {"state": state}
        self.send_cmd("set_network_bridge", args)

    @need('bluetooth')
    def get_bt_tethering_power(self):
        """
        Get Bluetooth Tethering feature state (Enable/disable).
        :rtype: boolean
        :return: True = enable
                 False = disable
        """
        self._logger.info("Get BT Tethering state")

        status, result = self.send_cmd("get_network_bridge_state", {})

        if status == CommandServerApi.SRV_CMD_FAILURE:
            msg = "Failed to get BT tethering state: %s" % result
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

        self._logger.info("BT tethering state: " + result)
        # pylint: disable=E1101
        if result == "ON":
            return True
        else:
            return False

    def set_bt_scanning(self, mode):
        """
        Starts/stops the device discovery session.
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        self._logger.info("Set BT scanning mode to %s" % str(mode))

        if mode in ['on', '1', 1]:
            mode = "ON"
        elif mode in ['off', '0', 0]:
            mode = "OFF"
        else:
            msg = "Invalid mode '%s'" % str(mode)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        status, result = self.send_cmd("set_scan_mode", {"mode": mode})

        if status == CommandServerApi.SRV_CMD_FAILURE:
            msg = "Failed to start BT discovery"
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

        self._logger.info(result)
