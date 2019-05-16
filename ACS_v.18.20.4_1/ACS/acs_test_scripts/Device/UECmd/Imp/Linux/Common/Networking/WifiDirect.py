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

:organization: NDG Validation
:summary: This file implements the Wifi Direct specific methods.
:since: 2014-10-18
:author: agoeax

"""
import re
from Device.UECmd.Imp.Linux.Common.Networking.Networking import Networking
from Device.UECmd.Interface.Networking.IWifiDirect import IWifiDirect
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from time import sleep


class WifiDirect(Networking, IWifiDirect):
    """
    Implementation of the Wifi Direct specific methods.
    """

    def __init__(self, device):
        """
        Constructor
        """
        Networking.__init__(self, device)
        self.__p2p_interface = "p2p-dev-wlan0"

    def enable_p2p(self):
        """
        Start WiFi Direct on dut.

        :rtype: None
        """
        if self.get_wifi_hotspot_status() == 1 and self._disable_one_time_setup:
            self._logger.debug("Disabling the One Time Setup mode")
            self._nw_module.disable_one_time_setup_mode()

        self._logger.debug("Starting WiFi Direct")

        if self.get_wifi_power_status() == 1 and self._check_p2p_enable():
            self._logger.info("Wifi Direct already started !")
            return

        cmd = self._start_wpa_sup
        self._internal_exec(cmd, use_uart=False)

        if self.get_wifi_power_status() == 1 and self._check_p2p_enable():
            self._logger.debug("Wifi Direct successfully started")
        else:
            DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "Unable to start WiFi Direct")

    def _check_p2p_enable(self):
        """
        Check the Wifi Direct network interface

        :rtype: Boolean
        :return p2p_enabled: True - P2P interface is up
                             False - P2P interface is down
        """
        cmd = "%s interface" % self._wpa_cli
        status, output = self._internal_exec(cmd, use_uart=False)
        # Results can look like:
        # root@edison:~# wpa_cli interface
        # Available interfaces:
        # p2p-dev-wlan0
        # wlan0
        lines = output.split('\n')
        p2p_enabled = False
        for line in enumerate(lines):
            if self.__p2p_interface in line:
                p2p_enabled = True

        return p2p_enabled

    def get_wifi_direct_interface(self):
        """
        Get the network interface for the P2P connection
        :rtype: String
        :return name: name of the network interface for the P2P connection
        """
        cmd = "%s -iwlan0 interface |grep p2p-wlan0" % self._wpa_cli
        status, output = self._internal_exec(cmd,raise_exception=False, use_uart=False)
        # Results can look like:
        # root@edison:~# wpa_cli -iwlan0 interface |grep p2p-wlan0
        # p2p-wlan0-1

        if "p2p-wlan0" in output:
            interface = output
            self._logger.info("name of the interface : " + interface)
        else:
            interface = ""
            self._logger.debug("no interface p2p-wlan0-x")

        return interface

    def wifi_direct_scan_peer_devices(self):
        """
        initiates a wifi direct discovery of the peers (scan)
        """
        cmd = "%s -i%s p2p_find " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)

        if "FAIL" in output:
            msg = "WiFi Direct scan failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)


    def wifi_direct_disconnect(self, wifi_direct_name=""):
        """
        Disconnect from a wifi direct peer device.

        :type wifi_direct_name: String
        :param wifi_direct_name: The name of the p2p connect interface you want to disconnect from.
        """
        if wifi_direct_name != "":
            cmd = "%s -i%s p2p_group_remove %s" % (self._wpa_cli, self.__p2p_interface, wifi_direct_name)
            status, output = self._internal_exec(cmd, use_uart=False)

            if "FAIL" in output:
                msg = "WiFi Direct peers list flush failed!"
                self._logger.error(msg)
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)
        else:
            msg = "No group to remove"
            self._logger.debug(msg)


    def wifi_direct_remove_remembered_groups(self):
        """
        Removes Wifi direct remembered groups
        """
        cmd = "%s -i%s remove_network all " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)

        if "FAIL" in output:
            msg = "WiFi Direct group removal failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    def wifi_direct_flush(self):
        """
        Removes Wifi direct connections
        """

        connect_interface = self.get_wifi_direct_interface()
        self.wifi_direct_disconnect(connect_interface)

        cmd = "%s -i%s p2p_flush " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)

        if "FAIL" in output:
            msg = "WiFi Direct peers list flush failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        cmd = "%s -i%s p2p_service_flush " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)

        if "FAIL" in output:
            msg = "WiFi Direct service flush failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        cmd = "%s -i%s save " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)

        if "FAIL" in output:
            msg = "WiFi Direct save failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)


    def _wifi_direct_wait_for_connection(self, timeout):
        """
        Waits for new connection interface to be created

        :type timeout: int
        :param timeout: Connection timeout
        """
        dev_connected = False
        remaining_time = timeout
        while (remaining_time > 0):
            #if self.get_wifi_direct_interface() is not None:
            if "p2p-wlan0" in self.get_wifi_direct_interface():
                self._logger.info("Peer Device is connected after %d s" % \
                                  (timeout - remaining_time))
                dev_connected = True
                break
            sleep(1)
            remaining_time -= 1
        if dev_connected is not True:
            msg = "WiFi Direct connection failed, timeout reached!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    def get_wifi_direct_mac_address(self):
        """
        Gets the device Wifi Direct mac address.

        :rtype: String
        :return mac_addr: the device Wifi Direct mac address
        """

        cmd = "%s -i%s status |grep p2p_device_address " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)

        if "FAIL" in output:
            msg = "Get WiFi Direct Mac Address failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        mac_list=output.split("=")
        mac_addr = mac_list[1]
        return mac_addr


    def get_wifi_direct_peer_devices(self, state="all"):
        """
        Gets the list of wifi direct peer devices scanned

        :type state: string (possible values all,available,invited,connected)
        :param state: If you want to get only the devices in a given state, use this field for filtering.
        :rtype: List of String
        :return name: list of wifi direct peer devices scanned
        """
        if state in ("connected", "all", "available", "invited"):
            cmd = "%s -i%s p2p_peers " % (self._wpa_cli, self.__p2p_interface)
        else:
            self._logger.debug("get_wifi_direct_peer_devices: %s unknown type of devices" % state)
            return None

        status, output = self._internal_exec(cmd, use_uart=False)
        list_peers=output.split('\n')
        return list_peers

    def wifi_direct_nw_config(self):
        """
        Configures the P2P network interface
        """
        cmd = "%s -i%s p2p_set listen_channel 11 " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)
        if "FAIL" in output:
            msg = "WiFi Direct config failure!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        cmd = "%s -i%s set persistent_reconnect 1 " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)
        if "FAIL" in output:
            msg = "WiFi Direct config failure!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        cmd = "%s -i%s p2p_set go_apsd 0 " % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)
        if "FAIL" in output:
            msg = "WiFi Direct config failure!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    def wifi_direct_discover(self):
        """
        Start P2P device discovery
        """
        cmd = "%s -i%s p2p_listen" % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)
        if "FAIL" in output:
            msg = "WiFi Direct discover failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    def wifi_direct_connect(self, peer_device_address="", timeout=40, go_intent=0, force_freq=0, connect_type="",
                            authentication_type="",
                            pin_code=""):
        """
        Connect to a wifi direct peer device.

        :type peer_device_address: String
        :param peer_device_address: The address of the device you want to connect to.
        :type timeout: int
        :param timeout: Connection timeout.
        :type go_intent: int
        :param go_intent: Group owner intent.
        :type force_freq: int
        :param force_freq: Frequency used for the communication (forced)
        :type connect_type: string
        :param connect_type: Type of connection (auto - connecting to an autonomous group
                                            intent - connecting using go_intent parameter set to 0)
        :type authentication_type: string
        :param authentication_type: Type of authentication  (WPS-PBC - connecting using PushButton
                                                   WPS-PIN - connecting using PIN code)
        :type pin_code: string
        :param pin_code: Pin code used when auth_type is WPS-PIN        .
        """
        self._wait_for_reported(peer_device_address, timeout)
        if connect_type == "intent":
            if go_intent >= 8:
                self._wifi_direct_connect_as_go(peer_device_address, authentication_type, pin_code, timeout,force_freq)
            else:
                self._wifi_direct_connect_as_cli(peer_device_address, connect_type, authentication_type, pin_code, timeout,force_freq)
        elif connect_type == "auto":
            self._wifi_direct_connect_as_cli(peer_device_address, connect_type, authentication_type, pin_code, timeout,force_freq)

    def _wait_for_reported(self, peer_address, timeout):
        """
        Waits for peer device to add REPORTED to "flags"

        :type timeout: int
        :param timeout: Connection timeout
        :type peer_address:
        :param peer_address: MAC address of the peer device
        """
        dev_reported = False
        remaining_time = timeout
        while (remaining_time > 0):
            cmd = "wpa_cli  -i%s p2p_peer %s|grep -c -e REPORTED -e GROUP_CLIENT_ONLY" % (
                self.__p2p_interface, peer_address)
            status, output = self._internal_exec(cmd,raise_exception=False, use_uart=False)
            if "1" in output:
                dev_reported = True
                break
            sleep(1)
            remaining_time -= 1
        if dev_reported is not True:
            msg = "WiFi Direct connection failed, timeout reached!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    def _wifi_direct_connect_as_cli(self, peer_device_address="", connect="", auth_type="", pin_code="", timeout=20,force_freq=0):
        """
        Connect to a wifi direct peer device when DUT is CLI.

        :type peer_device_address: String
        :param peer_device_address: The address of the device you want to connect to.
        :type connect: string
        :param connect: Type of connection (auto - connecting to an autonomous group
                                            intent - connecting using go_intent parameter set to 0)
        :type auth_type: string
        :param auth_type: Type of authentication  (WPS-PBC - connecting using PushButton
                                                   WPS-PIN - connecting using PIN code)
        :type pin_code: string
        :param pin_code: Pin code used when auth_type is WPS-PIN
        :type timeout: int
        :param timeout: Connection timeout.
        :type force_freq: int
        :param force_freq: p2p connection frequency.
        """
        cmd = "%s -i%s p2p_connect %s" % (self._wpa_cli, self.__p2p_interface, peer_device_address)
        if "WPS-PBC" in auth_type:
            cmd += " pbc"
        elif "WPS-PIN" in auth_type and pin_code != "":
            cmd += " %s keypad" % pin_code
        else:
            msg = "Wrong authentication type given (%s)" % auth_type
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        cmd += " persistent"

        if "auto" in connect:
            cmd += " join"
        elif "intent" in connect:
            if force_freq != 0:
                cmd += " freq=%d" % force_freq

            cmd += " go_intent=0"
        else:
            msg = "Wrong connection type given (%s)" % connect
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        sleep(5)
        status, output = self._internal_exec(cmd, use_uart=False)
        sleep(10)
        if "FAIL" in output:
            msg = "WiFi Direct connection failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)
        else:
            self._logger.info("Waiting up to %d s for connection to peer" % timeout)
            self._wifi_direct_wait_for_connection(timeout)

    def _wifi_direct_connect_as_go(self, incoming_peer="", auth_type="", pin_code="", timeout=20,force_freq=0):
        """
        Connects to a peer device with DUT as GO.

        :type incoming_peer: string
        :param incoming_peer: MAC address of the peer you want
        """

        cmd = "%s -i%s p2p_connect %s" % (self._wpa_cli, self.__p2p_interface, incoming_peer)
        if "WPS-PBC" in auth_type:
            cmd += " pbc"
        elif "WPS-PIN" in auth_type and pin_code != "":
            cmd += " %s display" % pin_code
        else:
            msg = "Wrong authentication type given (%s)" % auth_type
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        cmd += " persistent"


        if force_freq != 0:
            cmd += " freq=%d" % force_freq

        cmd += " go_intent=15"
        sleep(5)
        status, output = self._internal_exec(cmd, use_uart=False)
        sleep(10)
        if "FAIL" in output:
            msg = "WiFi Direct connection failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)
        else:
            self._logger.info("Waiting up to %d s for connection to peer" % timeout)
            self._wifi_direct_wait_for_connection(timeout)

    def wifi_direct_create_group(self, group_freq="", auth_type=""):
        """
        Creates a Autonomous Group, forcing the DUT to be GO.

        :type group_freq: string
        :param group_freq: Frequency of the Autonomous Group
        :type auth_type: string
        :param auth_type: Type of authentication  (WPS-PBC - connecting using PushButton
                                                   WPS-PIN - connecting using PIN code)
        :rtype: String
        :return name: Pin code if auth_type=WPS_PIN
        """
        cmd = "%s -i%s p2p_set ssid_postfix abc" % (self._wpa_cli, self.__p2p_interface)
        self._internal_exec(cmd, use_uart=False)
        if group_freq != "":
            cmd = "%s -i%s p2p_group_add persistent freq=%s" % (self._wpa_cli, self.__p2p_interface, group_freq)
            self._internal_exec(cmd, use_uart=False)
        else:
            msg = "Wrong group frequency given"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # wait 2 sec for the group to be created
        sleep(2)

        self._wifi_direct_wait_for_connection(20)

        group_interface = self.get_wifi_direct_interface()


        # set the type of authentication
        output=""
        if "WPS-PBC" in auth_type:
            cmd = "wpa_cli  -i%s wps_pbc" % group_interface
            self._internal_exec(cmd, use_uart=False)
        elif "WPS-PIN" in auth_type:
            cmd = "wpa_cli  -i%s wps_pin any" % group_interface
            status, output = self._internal_exec(cmd, use_uart=False)

            if not output.isdigit():
                msg="Not a PIN code : %s" % output
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        else:
            msg = "Wrong authentication type given"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        self.wifi_direct_scan_peer_devices()

        return output

    def get_wifi_direct_remembered_groups(self):
        """
        Gets the Wifi direct remembered groups.
        :rtype: List of String
        :return: list of wifi direct remembered groups.
        """
        cmd = "%s -i%s list_network" % (self._wpa_cli, self.__p2p_interface)
        status, output = self._internal_exec(cmd, use_uart=False)
        return output

    def wifi_direct_invite(self,peer_device="",force_freq=0):
        """
        Specific Edison

        Invite a peer to join a group or to reinvoke a persistent group.

        :type peer_device_address: String
        :param peer_device_address: The address of the device you want to connect to.
        :type force_freq: int
        :param force_freq: Frequency used for the communication (forced)
        """
        self._wait_for_reported(peer_device, 10)

        cmd = "%s -i%s p2p_invite persistent=0" % (self._wpa_cli, self.__p2p_interface)

        cmd += " peer=%s" % peer_device

        if force_freq != 0:
            cmd += " freq=%d" % force_freq

        status, output = self._internal_exec(cmd, use_uart=False)

        if "FAIL" in output:
            msg = "WiFi Direct connection failed!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)
        else:
            self._logger.info("Waiting up to %d s for connection to peer" % 20)
            self._wifi_direct_wait_for_connection(20)


    def get_wifi_direct_mode(self, wifi_direct_interface):
        """
        Gets the WiFi direct mode (CLI/GO).
        :type wifi_direct_interface: String
        :param wifi_direct_interface: Name of the WiFi Direct interface.
        :rtype: str
        :return: WiFi Direct mode
        """

        cmd = "%s -i%s status |grep mode" % (self._wpa_cli, wifi_direct_interface)
        status, output = self._internal_exec(cmd,raise_exception=False, use_uart=False)
        # Results can look like:
        # root@edison:~# wpa_cli -ip2p-wlan0-1 status |grep mode
        # mode=P2P GO or mode=station

        if "GO" in output:
            return "GO"
        elif "station" in output:
            return "CLI"
        else:
            msg = "Unknown WiFi Direct mode : %s" % output
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
