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

:organization: INTEL NDG SW
:summary: This file implements Networking UECmds for Linux device
:since: 04/29/2014
:author: jreynaux
"""
import time
import re
import random
import os
import subprocess
from string import digits
from Queue import Empty

import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from UtilitiesFWK.Utilities import Global, run_local_command, split_and_strip
from acs_test_scripts.Utilities.IPerfUtilities import \
    IperfOptions, IperfExecutionHandler, search_for_mandatory_keyword_arguments
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiKeyExchangeTypes
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.Networking.INetworking import INetworking
from acs_test_scripts.Device.UECmd.UECmdTypes import Measure, AUTO_CONNECT_STATE
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.NetworkingUtilities import SupplicantState
from acs_test_scripts.Utilities.IPerfUtilities import Iperf
import threading
from Queue import Queue


class Networking(Base, INetworking, Iperf):

    """
    Class that handle all networking operations
    """

    # Wifi frequencies bands supported by Edison.
    SUPPORTED_WIFI_BANDS = {"auto": 0, "5GHz": 1, "2.4GHz": 2}

    # Constant values for WIFI_SLEEP_POLICY
    WIFI_SLEEP_POLICY = {"DEFAULT": 0x0, "WHEN_SCREEN_OFF": 0x0,
                         "NEVER": 0x2, "NEVER_WHILE_PLUGGED": 0x1}

    def __init__(self, device):
        """
        Constructor
        """
        Base.__init__(self, device)
        INetworking.__init__(self, device)
        Iperf.__init__(self)

        self._device = device
        self._logger = self._device.get_logger()

        self._file_api = self._device.get_uecmd("File")
        self._phone_system = device.get_uecmd("PhoneSystem")

        # Get nw module to get hardware specific config
        self._nw_module = self._device.get_device_module("NetworkingModule")
        self._nw_module.init()
        # Var related to OneTimeSetup
        self._disable_one_time_setup = self._nw_module.networking_properties.disable_one_time_setup

        self._wpa_cli = "/usr/sbin/wpa_cli"
        self._start_wpa_sup = "/bin/systemctl start wpa_supplicant"
        self._stop_wpa_sup = "/bin/systemctl stop wpa_supplicant"
        self.__lock_file = "/var/run/wpa_supplicant/wlan0"
        self.__dhcp_stop = "/bin/systemctl stop wpa_supplicant_wlan0_event"
        self.__iperf = "/usr/sbin/iperf"
        self.__ssh = "ssh root@%s " % self._device.get_device_ip()
        self.__resolv = "/run/systemd/network/resolv.conf"
        self.__tls_prv_key = "123456"
        self.__tls_cerf = "/etc/ca-certificates/"
        self.__ip_retrieve_delay = 30
        # Remember last configured regulation domain
        self.__last_set_reg_domain = ""
        self.__wifi_freq_band = ""
        self.__flight_mode = 0

        self.__get_hostapd_status = "/bin/systemctl status hostapd"
        self.__start_hostapd = "/bin/systemctl start hostapd"
        self.__stop_hostapd = "/bin/systemctl stop hostapd"
        self.__disable_hostapd = "/bin/systemctl disable hostapd"
        self.__proc = None
        self.__q = None

    @need('wifi', False)
    def set_wifi_power(self, mode):
        """
        Sets the WIFI power to off or on.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :raise AcsConfigException if wrong parameter given

        :return: None
        """
        if mode in ["on", "1", 1]:
            self._start_wifi()
        elif mode in ["off", "0", 0]:
            self._stop_wifi()
        else:
            msg = "Wrong MODE given (%s)" % str(mode)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    @need('wifi')
    def set_wifi_frequency_band(self, freq_band, silent_mode=False, interface="wlan0"):
        """
        Set the Wifi Frequency Band

        :type freq_band: String
        :param freq_band: Frequency Band to set (auto, 2.4GHz, 5GHz)

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception
                            if the device does not support this method

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        """
        self.__wifi_freq_band = freq_band
        self._logger.debug("set_wifi_frequency_band: Not used on Linux devices")

    @need('wifi')
    def get_wifi_frequency_band(self, silent_mode=False, interface="wlan0"):
        """
        Gets the band selection (bands of frequencies)
        0 means dual
        1 means 5Ghz
        2 means 2.4Ghz

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception if the device does not support this method

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: String
        :return: The band text.
        """
        return self.__wifi_freq_band

    @need('wifi')
    def request_wifi_scan(self):
        """
        Trigger a Wifi scan.

        :return: None
        """
        cmd = "%s scan" % self._wpa_cli
        status, output = self._internal_exec(cmd, use_uart=False)
        if "FAIL" in output:
            msg = "Unable to request scan ! (%s)" % str(output)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    @need('wifi', False, 0)
    def get_wifi_power_status(self):
        """
        Gets the WIFI power.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        power = 42
        status, output = self._internal_exec("pidof wpa_supplicant", raise_exception=False, use_uart=False)

        if status and str(output) != "":
            if output.isdigit():
                self._logger.debug("Wifi is ON")
                power = 1
        elif not status and output == "":
            self._logger.debug("Wifi is OFF")
            power = 0
        else:
            msg = "Unable to get wifi power status ({0})".format(output)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        return power

    def _get_dhcp_client_status(self):
        """
        Gets the WIFI power.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        status, output = self._internal_exec("pidof dhcpc", use_uart=False)
        if status:
            if str(output) != "" and output.isdigit():
                self._logger.debug("DHCP client is ON")
                power = 1
            else:
                self._logger.debug("DHCP client is OFF")
                power = 0
        else:
            msg = "Unable to get DHCP client status"
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        return power

    def _get_wifi_connect_status(self):
        """
        Gets the status of the Wifi connection .

        :rtype: string
        :return: connection status
        """
        cmd = "%s status" % self._wpa_cli
        status, output = self._internal_exec(cmd, use_uart=False)

        # Result looks like:
        # Selected interface 'wlan0'
        # bssid=34:08:04:e1:8a:7c
        # ssid=EdisonTest
        # id=0
        # mode=station
        # pairwise_cipher=CCMP
        # group_cipher=TKIP
        # key_mgmt=WPA-PSK
        # wpa_state=COMPLETED
        # address=78:4b:87:91:6d:18
        # uuid=9e2a17bc-01e4-52a8-ac55-2b9f7da822bd

        connect = ""
        ssid = ""

        lines = output.split('\n')
        for line in lines:
            if "wpa_state" in line:
                connect = str(line).split('=')[1]

            if "ssid" in line:
                ssid = str(line).split('=')[1]

        if connect == "":
            msg = "Unable to get wifi connection status"
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        return ssid, connect

    @need('wifi')
    def _get_raw_networks_list(self):
        """
        Gets the full raw list of SSIDS and infos after a scan.
         This utility method, can be used by others Networking methods as list_ssids()

        :rtype: list
        :return: The raw list of str describing ssids, frequencies and capabilities.
        """
        # Get scan results
        cmd = "%s scan_results" % self._wpa_cli
        status, output = self._internal_exec(cmd, use_uart=False)
        # Results look like:
        # Selected interface 'wlan0'
        # bssid / frequency / signal level / flags / ssid
        # 24:01:c7:3b:5a:d0	2412	-51	[WPA2-PSK-CCMP][ESS]	CELAD WIFI
        # 24:01:c7:90:00:70	2437	-58	[WPA2-PSK-CCMP][ESS]	CELAD WIFI
        # so remove first 2 lines
        lines = output.split('\n')[2:]
        return lines

    @need('wifi')
    def _get_raw_configured_wifi_list(self):
        """
        Gets the full raw list of SSIDS and infos registered on system.
         This utility method, can be used by others Networking methods as list_connected_wifi()

        :rtype: list
        :return: The raw list of str describing ids, ssids, bssid and flags.
        """
        # Get list results
        cmd = "%s list_network" % self._wpa_cli
        status, output = self._internal_exec(cmd, use_uart=False)
        # Results can look like:
        # root@edison:~# wpa_cli list_network
        # Selected interface 'wlan0'
        # network id / ssid / bssid / flags
        # 0		any	[DISABLED]
        # or
        # 0	INTELTEST_NOSEC	any	[DISABLED]
        # or
        # 0	INTELTEST_NOSEC	any
        # or
        # 0	INTELTEST_NOSEC	any	[CURRENT]
        # so remove first 2 lines
        lines = output.split('\n')[2:]
        return lines

    @need('wifi')
    def _start_wifi(self):
        """
        Start wpa_supplicant on dut.

        :rtype: None
        """
        if self.get_wifi_hotspot_status() == 1 and self._disable_one_time_setup:
            self._logger.debug("Disabling the One Time Setup mode")
            self._nw_module.disable_one_time_setup_mode()

        self._logger.debug("Starting wpa_supplicant")

        if self.get_wifi_power_status() == 1:
            self._logger.info("Wifi (wpa_supplicant) already started !")
            return

        cmd = self._start_wpa_sup
        self._internal_exec(cmd, use_uart=False)

        if self.get_wifi_power_status() == 1:
            self._logger.debug("Wifi (wpa_supplicant) successfully started")
        else:
            DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "Unable to start wifi")

    @need('wifi')
    def _stop_wifi(self):
        """
        Stop the current wpa_supplicant process, then remove the ctrl_iface lock file

        :rtype: None
        """
        kill_status = True
        self._logger.debug("Stopping wpa_supplicant")

        if self.get_wifi_power_status() == 0:
            msg = "Wifi (wpa_supplicant) already stopped !"
            self._logger.info(msg)
            return

        cmd = self._stop_wpa_sup
        self._internal_exec(cmd, use_uart=False)

        if self.get_wifi_power_status() == 0:
            self._logger.debug("Wifi (wpa_supplicant) successfully stopped")
        else:
            msg = "Unable to stop wifi (wpa_supplicant) !"
            self._logger.warning(msg)
            kill_status = False

        del_status, o = self._file_api.delete(self.__lock_file)

        status = del_status if kill_status is False else True

        if not self._file_api.exist(self.__lock_file)[0]:
            self._logger.debug("ctrl_iface successfully deleted")
        else:
            self._logger.warning("unable to delete ctrl_iface !")

        if not status:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "Unable to stop wifi")

    def _stop_dhcp(self):
        """
        Stop the current dhcp client process

        :rtype: None
        """
        self._logger.debug("Stopping DHCP client")

        status, output = self._internal_exec(self.__dhcp_stop, use_uart=False)

        if not status:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "Unable to stop DHCP")

    @need('wifi')
    def _get_wifi_index_from_ssid(self, ssid):
        """
        Get the wifi index on configured networks.

        Used for wifi network connection management

        :type ssid: str
        :param ssid: Ssid of network to found

        :rtype: int
        :return: The Id of ssid to found, None if not found
        """
        # Get configured networks
        cmd = "%s list_network" % self._wpa_cli
        status, output = self._internal_exec(cmd, use_uart=False)
        # Results can look like:
        # root@edison:~# wpa_cli list_network
        # Selected interface 'wlan0'
        # network id / ssid / bssid / flags
        # 0		any	[DISABLED]
        # so remove first 2 lines
        lines = output.split('\n')[2:]
        index = None
        for count, line in enumerate(lines):
            if str(ssid) in line:
                index = int(line[:1])
                self._logger.debug('Network %s found as ssid %d' % (str(ssid), index))
                break

        if index is None:
            self._logger.error('Unable to find network %s on listed network !' % str(ssid))

        return index

    @need('wifi')
    def list_ssids(self, technology="wifi", state="all"):
        """
        Lists the ssids for a given technology and a given state.
        .. warning:: technology parameter unused (only wifi ssids available).

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :type state: str
        :param state: the state of the ssid to list value can be
                         - "connected"
                         - "disconnected"
                         - "all" for any state

        :rtype: list
        :return: a list of ssids
        """
        if technology not in ("wifi", "all"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "%s technology not supported !" % technology)

        if state not in ("connected", "disconnected", "all", "remembered", "visible"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "%s state not supported !" % state)

        if "connected" in state:
            return self.list_connected_wifi()

        networks = []
        if state in ("remembered", "all"):
            raw_network = self._get_raw_configured_wifi_list()
            for count, line in enumerate(raw_network):
                # Format of each line:
                # bssid / frequency / signal level / flags / ssid
                network = line.split('\t')
                # Can be stored as dict, like:
                # n = {'network id': network[0], 'ssid': network[1],
                #       'flags': network[2], 'state': network[3]}
                # Get only SSIDs
                n = ""
                # Check if there an ssid (maybe an hidden ssid at end of list)
                n = str(network[1])
                self._logger.debug("Network %d: %s" % (count+1, n if n != "" else "???"))
                networks.append(n)
        if state in ("visible", "all") :
            raw_network = self._get_raw_networks_list()
            for count, line in enumerate(raw_network):
                # Format of each line:
                # bssid / frequency / signal level / flags / ssid
                network = line.split('\t')
                # Can be stored as dict, like:
                # n = {'bssid': network[0], 'frequency': network[1],
                #       'signal_level': network[2], 'flags': network[3], 'ssid': network[4]}
                # Get only SSIDs
                n = ""
                # Check if there an ssid (maybe an hidden ssid at end of list)
                if len(network) >= 5:
                    n = str(network[4])
                self._logger.debug("Network %d: %s" % (count+1, n if n != "" else "???"))
                networks.append(n)

        return networks

    @need('wifi')
    def list_connected_wifi(self):
        """
        Lists the connected WIFI.

        :rtype: list
        :return: the requested value
        """
        raw_connected_network = self._get_raw_configured_wifi_list()
        # network id / ssid / bssid / flags
        expr = "^([0-9])\s+([-_0-9a-zA-Z\s]*)\s+([a-z]+)\s*\[*([a-zA-Z]*)\]*.*$"

        connected_network = []
        for count, line in enumerate(raw_connected_network):
            m = re.match(expr, line)
            if m is not None:
                nw_id = m.group(1)
                ssid = m.group(2)
                bssid = m.group(3)
                flag = m.group(4)
                self._logger.debug("network id: {0}, ssid: {1}, bssid: {2}, flag: {3}"
                    .format(nw_id, ssid, bssid, flag))
                if ssid != '' and flag == 'CURRENT':
                    connected_network.append(str(ssid))
        return connected_network

    def set_autoconnect_mode(self, interface, state):
        """
        Sets the autoconnect mode to on/off for a specific interface or all

        :type interface: String
        :param interface: interface to modify or 'all' for all interfaces

        :type state: str or int
        :param state: Can be AUTO_CONNECT_STATE.on  to enable autoconnect
                             AUTO_CONNECT_STATE.off to disable autoconnect

        :return: None
        """

        if state not in AUTO_CONNECT_STATE:
            # Unknown request state
            output = "set_autoconnect_mode : %s is not in known state" % state
            self._logger.error(output)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, output)
        elif state == AUTO_CONNECT_STATE.on:
            self._logger.debug("Setting auto-connect to ON")
            network_list = self._get_raw_configured_wifi_list()
            for network in network_list:
                if interface in network:
                    self.wifi_connect(interface)


    @need('wifi')
    def set_wificonfiguration(self, ssid="UNKNOWN", pass_phrase=None, security="UNKNOWN",
                              ip_method="", address="", netmask="",
                              gateway="", dns1="", dns2="", proxy_config="NONE",
                              proxy_address="", proxy_port="", proxy_bypass=None):

        """
        Sets passphrase for secured WIFI network

        :type ssid: str
        :param ssid: WIFI network SSID

        :type pass_phrase: str
        :param pass_phrase: Passphrase

        :type security: str
        :param security : wifi security name. Can be NONE, OPEN, WEP, WPA, WPA2,
         WEP64-OPEN, WEP128-OPEN, WEP64, WEP128, WPA2-PSK-AES, WPA-PSK-TKIP,
         WPA2-PSK-TKIP, WPA-PSK-AES, WPA-PSK, WPA-PSK-TKIP-AES,
         WPA2-PSK, WPA2-PSK-TKIP-AES, WPA-WPA2-PSK, WPA-WPA2-PSK-TKIP-AES,
         WPA-WPA2-PSK-TKIP, "WPA-WPA2-PSK-AES

        :type ip_method: str
        :param ip_method: Possible values are "dhcp" or "static"

        :type address: str
        :param address: The current configured IPv4 address.

        :type netmask: str
        :param netmask: The current configured IPv4 netmask.

        :type gateway: str
        :param gateway: The current configured IPv4 gateway.

        :type dns1: str
        :param dns1: The current IPV4 dns1 to configure.

        :type dns2: str
        :param dns2: The current IPV4 dns2 to configure.

        :type proxy_config: str
        :param proxy_config: Possible values are "NONE", "MANUAL" or "AUTO"

        :type proxy_address: str
        :param proxy_address: Address of the proxy

        :type proxy_port: str
        :param proxy_port: Port of the proxy

        :type proxy_bypass: str
        :param proxy_bypass: Addresses bypassed by the proxy separated by a comma

        :return: None
        """
        nw_index = None
        self._logger.info("Setting wifi configuration %s ..." % str(ssid))

        self._logger.debug("Add new network ...")
        cmd = "%s add_network" % self._wpa_cli
        status, output = self._internal_exec(cmd, use_uart=False)
        if status and output != "":
            o = output.split("\n")[1:][0]
            nw_index = int(o)

        if nw_index is not None:

            self._logger.debug("Set ssid %s to network %d ..." % (ssid, nw_index))
            # Use '"<ssid>"' format
            self._internal_exec("%s set_network %d ssid '\"%s\"'" % (self._wpa_cli, nw_index, ssid), use_uart=False)

            self._internal_exec("%s set_network %d scan_ssid 1" % (self._wpa_cli, nw_index), use_uart=False)

            key_mgmt = self.__get_wifi_security_type(security)

            self._logger.debug("Set security %s to network %d ..." % (security, nw_index))
            # Use <security> format
            self._internal_exec("%s set_network %d key_mgmt %s" % (self._wpa_cli, nw_index, key_mgmt), use_uart=False)

            if 'WPA2' in security:
                self._logger.debug("Set option WPA2 to network %d ..." % nw_index)
                # Use <security> format
                self._internal_exec("%s set_network %d proto WPA2" % (self._wpa_cli, nw_index), use_uart=False)

            if "EAP" in security and pass_phrase not in ["", "None", None]:
                self._logger.debug("Set EAP configuration to network %d ..." % (nw_index))
                self.__eap_config(nw_index, pass_phrase)
            else:
                if 'WEP' in security:
                        if 'OPEN' in security:
                            self._logger.debug("Set option auth_alg %s to network %d ..." % ("OPEN", nw_index))
                            # Use <WEP_TYPE> format
                            self._internal_exec("%s set_network %d auth_alg OPEN" % (self._wpa_cli, nw_index), use_uart=False)
                        else:
                            self._logger.debug("Set option auth_alg %s to network %d ..." % ("SHARED", nw_index))
                            # Use <WEP_TYPE> format
                            self._internal_exec("%s set_network %d auth_alg SHARED" % (self._wpa_cli, nw_index), use_uart=False)

                if pass_phrase not in ["", "None", None]:
                    if 'WEP' in security:
                        self._logger.debug("Set wep_key0 %s to network %d ..." % (pass_phrase, nw_index))
                        # Use '"<wep_key>"' format
                        self._internal_exec("%s set_network %d wep_key0 '\"%s\"'" % (self._wpa_cli, nw_index, pass_phrase), use_uart=False)
                    else :
                        self._logger.debug("Set password %s to network %d ..." % (pass_phrase, nw_index))
                        # Use '"<pass_phrase>"' format
                        self._internal_exec("%s set_network %d psk '\"%s\"'" % (self._wpa_cli, nw_index, pass_phrase), use_uart=False)

            self._internal_exec("%s save" % self._wpa_cli, use_uart=False)

            if ip_method.lower() in "static":
                self._stop_dhcp()
                self._set_interface_ipv4_address("wlan0",address, netmask, gateway, dns1, dns2)

        else:
            msg = "Unable to create new network configuration"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    def __eap_config(self, nw_index, pass_phrase):
        """
        Configure network connection in case of EAP type authentication.

        :type pass phrase : str
        :param pass_phrase: eap_method-phase2_auth_eap_user_eap_password_
                            certificat_name_str(use_certificate)
        """
        eap = split_and_strip(pass_phrase, "-")
        eap_method = eap[0]
        param = split_and_strip(eap[1], "_")
        phase2_auth = param[0]
        eap_user = param[1]
        eap_password = param[2]
        certificat_name = param[3]
        mandatory_cert = param[4]

        if eap_method in ("PEAP", "TLS", "TTLS", "FAST"):
            self._logger.debug("Set EAP option %s to network %d ..." % (eap_method, nw_index))
            # Use <eap_method> format
            self._internal_exec("%s set_network %d eap %s" % (self._wpa_cli, nw_index, eap_method), use_uart=False)

        if phase2_auth in ("MD5", "PAP", "MSCHAP", "MSCHAPV2", "GTC"):
            self._logger.debug("Set EAP option %s to network %d ..." % (phase2_auth, nw_index))
            # Use <eap_method> format
            self._internal_exec("%s set_network %d phase2 '\"auth=%s\"'" % (self._wpa_cli, nw_index, phase2_auth), use_uart=False)

        self._logger.debug("Set EAP anonymous_identity %s to network %d ..." % ("", nw_index))
        # Set "anonymous_identity"
        self._internal_exec("%s set_network %d anonymous_identity '\"%s\"'" % (self._wpa_cli, nw_index, ""), use_uart=False)

        if eap_user is not None:
            self._logger.debug("Set EAP user %s to network %d ..." % (eap_user, nw_index))
            # Use <eap_user> format
            self._internal_exec("%s set_network %d identity '\"%s\"'" % (self._wpa_cli, nw_index, eap_user), use_uart=False)

        if eap_method in "TLS" or mandatory_cert.lower() in "true":
            if certificat_name is not None:
                self._logger.debug("Set EAP ca_certificate %s_cacert to network %d ..." % (certificat_name, nw_index))
                self._internal_exec("%s set_network %d ca_cert '\"%s%s_cacert.pem\"'" % (self._wpa_cli, nw_index, self.__tls_cerf, certificat_name), use_uart=False)

                self._logger.debug("Set EAP client_certificate %s_ccert to network %d ..." % (certificat_name, nw_index))
                self._internal_exec("%s set_network %d client_cert '\"%s%s_ccert.pem\"'" % (self._wpa_cli, nw_index, self.__tls_cerf, certificat_name), use_uart=False)

                self._logger.debug("Set EAP private_key %s_key to network %d ..." % (certificat_name, nw_index))
                self._internal_exec("%s set_network %d private_key '\"%s%s_pkey.pem\"'" % (self._wpa_cli, nw_index, self.__tls_cerf, certificat_name), use_uart=False)

                self._logger.debug("Set EAP private_key_passwd '\"%s\"' to network %d ..." % (self.__tls_prv_key, nw_index))
                self._internal_exec("%s set_network %d private_key_passwd '\"%s\"'" % (self._wpa_cli, nw_index, self.__tls_prv_key), use_uart=False)
        else:
            if eap_password is not None:
                self._logger.debug("Set EAP password %s to network %d ..." % (eap_password, nw_index))
                # Use <eap_password> format
                self._internal_exec("%s set_network %d password '\"%s\"'" % (self._wpa_cli, nw_index, eap_password), use_uart=False)

        if eap_method in "FAST":
            self._logger.debug("Set EAP phase1 to network %d ..." % nw_index)
            self._internal_exec("%s set_network %d phase1 '\"fast_provisioning=2\"'" % (self._wpa_cli, nw_index), use_uart=False)

            self._logger.debug("Set EAP pac_file to network %d ..." % nw_index)
            self._internal_exec("%s set_network %d pac_file '\"blob://eap-fast-pac\"'" % (self._wpa_cli, nw_index), use_uart=False)

    def __get_wifi_security_type(self, security_parameter):
        """
        Get the Linux security type to set on device from the given security.

        :type security_parameter : str
        :param security_parameter: "normalized" security name. Can be NONE, OPEN, WEP, WPA, WPA2,
         WEP64-OPEN, WEP128-OPEN, WEP64, WEP128, WPA2-PSK-AES, WPA-PSK-TKIP,
         WPA2-PSK-TKIP, WPA-PSK-AES, WPA-PSK, WPA-PSK-TKIP-AES,
         WPA2-PSK, WPA2-PSK-TKIP-AES, WPA-WPA2-PSK, WPA-WPA2-PSK-TKIP-AES,
         WPA-WPA2-PSK-TKIP, "WPA-WPA2-PSK-AES

        :rtype: str
        :return: Linux specific security name
        """
        # Extract the security Type
        security_upper = str(security_parameter).upper()
        if security_upper in ("NONE", "OPEN"):
            security = "NONE"
        elif security_upper in ("WEP", "WEP64", "WEP128"):
            security = "NONE"
        elif security_upper in ("WEP64-OPEN", "WEP128-OPEN"):
            security = "NONE"
        elif security_upper in ("WPA", "WPA2", "WPA2-PSK-AES", "WPA-PSK-TKIP",
                                "WPA2-PSK-TKIP", "WPA-PSK-AES",
                                "WPA-PSK", "WPA-PSK-TKIP-AES",
                                "WPA2-PSK", "WPA2-PSK-TKIP-AES",
                                "WPA-WPA2-PSK", "WPA-WPA2-PSK-TKIP-AES",
                                "WPA-WPA2-PSK-TKIP", "WPA-WPA2-PSK-AES"):
            security = "WPA-PSK"
        elif security_upper in ("EAP-WPA", "EAP-WPA2", "EAP-WPA-WPA2"):
            security = "WPA-EAP"
        else:
            msg = "Unexpected Wlan security %s!" % str(security_upper)
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

        return security

    @need('wifi')
    def wifi_remove_config(self, ssid):
        """
        Remove a wifi configuration for the device.

        :type ssid: str
        :param ssid: the ssid of the wifi configuration to be removed or "all"
        """
        if ssid in "all":
            list = self._get_raw_configured_wifi_list()
            for line in list:
                index = int(line[:1])
                self._wifi_remove_network_index(index)
        else:
            self._logger.info("Remove wifi network config for %s" % str(ssid))
            index = self._get_wifi_index_from_ssid(ssid)
            if index is not None:
                self._wifi_remove_network_index(index)

        self._internal_exec("%s save" % self._wpa_cli, use_uart=False)

    def _wifi_remove_network_index(self, index):
        """
        Remove a wifi configuration for the device.

        :type index: int
        :param index: index number of the network configuration to be removed
        """
        self._logger.info("Remove wifi network config index %d" % index)
        if index is not None:
            status, output = self._internal_exec("%s remove_network %d" % (self._wpa_cli, index), use_uart=False)

        if status and "OK" in output:
            self._logger.debug("Remove wifi network config index %d OK" % index)
        else:
            msg = "Unable to remove network configuration \"%s\"" % str(index)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

    @need('wifi')
    def wifi_connect(self, ssid, check_connection=True, poor_connection_check=False):
        """
        Connects to a WIFI network using its SSID

        :type ssid: str
        :param ssid: WIFI network SSID
        :type check_connection: boolean
        :param check_connection: if connection is checked
        :type poor_connection_check: Boolean
        :param poor_connection_check: Disables (False) or enables (True) the poor connection check (JB or later).

        :return: None
        """
        self._logger.info("Connecting to wifi network %s" % str(ssid))
        index = self._get_wifi_index_from_ssid(ssid)
        if index is not None:
            self._internal_exec("%s select_network %d" % (self._wpa_cli, index), use_uart=False)
        else:
            msg = "No valid network configuration found for ssid {0}".format(ssid)
            self._logger.error(msg + " Ensure is well configured before trying to enabling it.")
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)

        if check_connection:
            self._logger.info("Check connection state ...")
            self.check_connection_state(ssid)

        self._logger.debug("Wait a while for IP interface retrieval")
        time.sleep(self.__ip_retrieve_delay)

        self._internal_exec("/sbin/ifconfig wlan0", raise_exception=False, use_uart=False)

    @need('wifi')
    def check_connection_state(self, ssid, timeout=0):
        """
        Checks that the connected SSID is the one passed as parameter

        :type ssid: str
        :param ssid: SSID name of the Wifi network the DUT should be connected
        :type timeout: int
        :param timeout: Customize the timeout
                        (when timeout = 0 --> default value)
        """
        # remove quotes around ssid (used when SSID contains a space)
        if ssid[0] == "'" and ssid[-1] == "'":
            ssid = ssid[1:-1]

        self._logger.info("Search connection state of network %s" % str(ssid))

        connection_state = False

        timeout = int(timeout)
        if timeout <= 0:
            timeout = self._uecmd_default_timeout
        start = time.time()

        while not connection_state and not start + timeout < time.time():
            time.sleep(2)

            # List connected SSIDs to check if the right ssid is connected
            connected_wifi_list = self.list_connected_wifi()

            if ssid in connected_wifi_list:
                self._logger.info("Wifi connected to network %s" % str(ssid))
                connection_state = True
                break

        if not connection_state:
            raise DeviceException(DeviceException.TIMEOUT_REACHED, "Wifi connect to %s timeout !" % ssid)

    @need('wifi')
    def wifi_disconnect_all(self):
        """
        Disconnects from all WIFI network

        :return: None
        """
        self._logger.info("Disconnecting from all wifi networks")

        cmd = "%s disconnect" % self._wpa_cli
        status, output = self._internal_exec(cmd, use_uart=False)

        if "OK" not in output:
            self._logger.warning("Error during disconnect")

    @need('wifi or ethernet')
    def ping(self,
             server_ip_address,
             packet_size=32,
             packet_count=5,
             interval=1,
             flood_mode=False,
             blocking=True,
             source_address=None):
        """
        Pings from the DUT a server on the bench. the protocol to use (IPv4 or IPv6) is
        computed automatically.

        :type server_ip_address: str
        :param server_ip_address: IP address to ping

        :type packet_size: integer
        :param packet_size: Packet size in bytes

        :type packet_count: integer
        :param packet_count: Number of packet to send

        :type interval: float
        :param interval: Interval in seconds between pings (only for IPv4)

        :type flood_mode: Boolean
        :param flood_mode: True if you want to use the ping in flood mode, False else.

        :type blocking: Boolean
        :param blocking: True if you want to return error when NETWORK UNREACHABLE, False else.

        :type source_address: str
        :param source_address: source IP address to use

        :rtype: Measure object
        :return: packet loss
        """
        self._logger.info("Ping address " + str(server_ip_address) +
                          " with " + str(packet_count) + " packets of " +
                          str(packet_size) + " bytes...")
        if packet_count > 0:
            timeout = int(packet_count) * 10
        else:
            self._logger.error("'packet count' must be an Integer > 0")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "'packet count' must be an Integer > 0")

        cmd = "ping" + \
            " -s " + str(packet_size) + \
            " -c " + str(packet_count) + \
            " " + str(server_ip_address)

        status, output = self._internal_exec(cmd, timeout, raise_exception=False, use_uart=False)

        output_lower = output.lower()
        packet_loss = Measure()

        if "network is unreachable" not in output_lower and "bad address" not in output_lower:
            output_match = re.search("received, ([0-9]+\.?[0-9]*)% packet loss", output_lower)

            if output_match is None:
                error = "Ping results are missing (Output = %s)" % output
                self._logger.error(error)
            else:
                # Parse output to get measured % of packets lost
                self._logger.debug("Parsed packet loss rate: " + str(output_match.groups()))
                packet_loss.value = float(output_match.group(1))
                packet_loss.units = "%"

        else:
            error = "Ping command returned an error message (Output = %s)" % output
            if blocking:
                self._logger.error(error)
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error)
            else:
                self._logger.info(error)
                packet_loss.value = -1
                return packet_loss

        return packet_loss

    def set_regulatorydomain(self, regulatory_domain, interface="wlan0"):
        """
        Set the Wifi Regulatory Domain

        :type regulatory_domain: String
        :param regulatory_domain: the regulatory domain to set (FR, GB, US...)

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        # Store set value for later restoration.
        self.__last_set_reg_domain = regulatory_domain

    def get_interface_mac_addr(self, interface="wifi"):
        """
        Returns the MAC address of the given interface.
        .. warning:: unused parameter 'interface'.

        :type interface: str
        :param interface: the interface name.

        :rtype: str
        :return: The dut mac address
        """
        if interface in ["wifi", "wlan0"]:
            interface = "wlan0"
        else:
            msg = "Invalid interface %s" % interface
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        cmd = "ifconfig " + interface
        status, output = self._internal_exec(cmd, use_uart=False)
        m = re.search(r"HWaddr ([0-9a-z:]+) ", output)
        if m is None:
            msg = "Could not get MAC address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        interface_mac = m.groups()[0]
        return interface_mac

    def wifi_disconnect(self, ssid):
        """
        Disconnects from a WIFI network using its SSID

        :param ssid: WIFI network SSID

        :return: None
        """
        self._logger.info("Disconnecting from wifi network %s" % str(ssid))
        connected_list = self.list_connected_wifi()
        for network in connected_list:
            if ssid in network:
                cmd = "%s disconnect" % self._wpa_cli
                self._internal_exec(cmd, use_uart=False)
                self._logger.debug("Wifi network %s is disconnected" % str(ssid))

    def get_regulatorydomain(self):
        """
        Get the Wifi Regulatory Domain

        :rtype: String
        :return: the regulatory domain (FR, GB, US... or "none")
        """

        #returning the last regulatory domain
        return self.__last_set_reg_domain

    def get_wifi_ip_address(self, IPV6=False):
        """
        Returns the Wifi ip address.

        :type IPV6 Boolean
        :param IPV6 False to get IPV4 address, True to get the IPV6 one

        :rtype: str
        :return: wifi ip address
        """
        if not IPV6:
            output = self.get_interface_ipv4_address("wlan0")
        else:
            self._logger.debug("Can not retrieve IPV6 address")

        return str(output)

    def get_interface_ipv4_address(self, interface):
        """
        Returns the ipv4 address of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: str
        :return: interface ip address
        """
        cmd = "ifconfig " + interface
        status, output = self._internal_exec(cmd, use_uart=False)
        m = re.search(r"inet addr:([0-9.]+) ", output)
        if m is None:
            msg = "Could not get IPV4 address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        interface_ip = m.groups()[0]
        return interface_ip

    def _set_interface_ipv4_address(self, interface, ip_address, netmask="255.255.255.0",
                                    gateway="0.0.0.0", dns1="0.0.0.0", dns2="0.0.0.0"):
        """
        Setts the ipv4 address of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type ip_address: str
        :param ip_address: new IP address (format xxx.xxx.xxx.xxx)

        :type ip_address: netmask
        :param ip_address: new netmask (format xxx.xxx.xxx.xxx)

        :type ip_address: gateway
        :param ip_address: new gateway (format xxx.xxx.xxx.xxx)

        :type ip_address: dns1
        :param ip_address: new dns server no 1 (format xxx.xxx.xxx.xxx)

         :type ip_address: dns2
         :param ip_address: new dns server no 2 (format xxx.xxx.xxx.xxx)

        """
        self._logger.debug("Setting IP address %s to interface %s" % (ip_address, interface))
        cmd = "ifconfig " + interface + " inet " + ip_address + " netmask "+ netmask
        self._internal_exec(cmd, raise_exception=False, use_uart=False)

        if gateway != "" and gateway != "0.0.0.0":
            self._logger.debug("Setting default gateway %s" % gateway)
            cmd = "route add default gw " + gateway
            self._internal_exec(cmd, raise_exception=False, use_uart=False)

        if dns1 != "" and dns1 != "0.0.0.0":
            # Check current config
            check = "cat %s | grep %s" % (self.__resolv, dns1)
            status, output = self._internal_exec(check, raise_exception=False, use_uart=False)
            if dns1 not in output:
                self._logger.debug("Setting DNS1 to %s" % dns1)
                cmd = 'echo "nameserver %s" >> %s' % (dns1, self.__resolv)
                self._internal_exec(cmd, raise_exception=False, use_uart=False)

        if dns2 != "" and dns2 != "0.0.0.0":
            # Check current config
            check = "cat %s | grep %s" % (self.__resolv, dns2)
            status, output = self._internal_exec(check)
            if dns2 not in output:
                self._logger.debug("Setting DNS2 to %s" % dns2)
                cmd = 'echo "nameserver %s" >> %s' % (dns2, self.__resolv)
                self._internal_exec(cmd, raise_exception=False, use_uart=False)

    def get_wifi_dhcp_state(self):
        """
        Returns the Wifi DHCP state according to android NetworkInfo.State.

        :rtype: str
        :return: wifi DHCP state
        """
        dhcp_status = "ERROR"

        cmd = "udhcpc -i wlan0 -n status"

        status, output = self._internal_exec(cmd, use_uart=False)

        if 'Sending discover...' in output and 'Sending select for' in output:
            dhcp_status = "CONNECTED"

        return str(dhcp_status)

    def wait_for_wifi_dhcp_connected(self, timeout=None):
        """
        Polls until the IP address has been acquired.
        Then, DHCP state will be "CONNECTED".
        At the end of the default timeout, an exception will be raised

        :type timeout: int
        :param timeout: max number of seconds to wait for IP address
        """

        dhcp_state = self.get_wifi_dhcp_state()
        if timeout is None:
            timeout = self._uecmd_default_timeout
        self._logger.info("Waiting for DHCP state to be \"CONNECTED\"...")
        start_time = time.time()
        while (dhcp_state != "CONNECTED") and ((time.time() - start_time) < timeout):
            dhcp_state = self.get_wifi_dhcp_state()
            time.sleep(1)

        if dhcp_state != "CONNECTED":
            new_msg = "Can't get IP address... Is there a DHCP server running?"
            self._logger.error(new_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, new_msg)

    def wifi_setkeyexchange(self, ssid, key_exchange_mode, key_exchange_pin=None,
                            simulate_faulty_connection=False, interface="wlan0"):
        """
        Sets key exchange mode (WPS for instance) for secured WIFI network.

        :type key_exchange_mode: str
        :param key_exchange_mode: Key exchange mode

        :type key_exchange_pin: str
        :param key_exchange_pin: PIN used by some key exchange modes (WPS_PIN_FROM_AP
        for instance)

        :type ssid: str
        :param ssid: WIFI network SSID

        :type simulate_faulty_connection: bool
        :param simulate_faulty_connection: If True, corrupts the data or wait for the
        AP's timeout to simulate a faulty simulation request

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        :return: None
        """
        wpa_cli_param = ""
        wpa_cli_pin = ""

        if key_exchange_mode == WifiKeyExchangeTypes.WPS_PBC:
            wpa_cli_param = "wps_pbc"
        elif key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_AP:
            wpa_cli_param = "wps_reg"
            wpa_cli_pin = str(key_exchange_pin)
        elif key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_DUT:
            wpa_cli_param = "wps_pin"
            wpa_cli_pin = str(key_exchange_pin)
        else:
            output = "wifi_setkeyexchange: %s is not a good mode" % key_exchange_mode
            self._logger.error(output)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, output)

        bssid = self._get_bssid_from_ssid(ssid)

        # Corrupt the data to simulate a faulty connection request.
        if simulate_faulty_connection:
            self._logger.info("Simulating faulty connection request...")

            if key_exchange_mode == WifiKeyExchangeTypes.WPS_PBC:
                # Make the AP go to timeout
                ap_timeout = 120
                self._logger.info("Waiting for the AP timeout %ds" % ap_timeout)
                time.sleep(ap_timeout)
            elif ((key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_AP) or
                 (key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_DUT)):
                # Corrupt the PIN code
                self._logger.info("Corrupting the PIN code")
                new_digit = "0"
                old_digit = wpa_cli_pin[0]
                while new_digit == old_digit:
                    new_digit = random.choice(digits)
                wpa_cli_pin = wpa_cli_pin.replace(str(old_digit), str(new_digit))

        self._logger.info("Setting key exchange %s (pin: \"%s\")" % (key_exchange_mode, wpa_cli_pin))
        # Remove previous configuration
        cmd = "%s wps_cancel" % self._wpa_cli
        self._internal_exec(cmd, timeout=5, use_uart=False)
        self.wifi_remove_config(ssid)

        # Start WPS on DUT.
        cmd = "%s %s %s %s" % (self.wpa_cli, wpa_cli_param, bssid, wpa_cli_pin)
        self._logger.debug("Launching supplicant " + cmd)
        results = self._internal_exec(cmd, timeout=5, use_uart=False)
        if len(results.split()) < 3:
            raise DeviceException(DeviceException.OPERATION_FAILED, results)
        if results.split()[3] not in ["OK", wpa_cli_pin]:
            raise DeviceException(DeviceException.OPERATION_FAILED, results)
        # Wait for the command to be taken into account into the DUT.
        time.sleep(15)

    def _get_bssid_from_ssid(self, ssid, interface=None):
        """
        Get BSSID from SSID
        :type ssid: str
        :param ssid: The access point's SSID

        :rtype: str
        :return: The access point's mac address (BSSID)

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        if not ssid:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, ssid)

        cmd = "%s scan_result" % self._wpa_cli
        results = self._internal_exec(cmd, timeout=5, use_uart=False)
        # Align all the carriage returns to \n to make it work on all OS and all adb connect method
        results = results.replace('\r', '\n')
        bssid = ''
        for line in results.split('\n'):
            if line.endswith(ssid):
                bssid = str(line).split()[0]
                break

        if not NetworkingUtil.is_valid_mac_address(bssid):
            msg = "Unable to get BSSID from SSID '%s' (result = %s)" % (ssid, bssid)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return bssid

    def get_wifi_connection_status_log(self, ssid, timeout=120):
        """
        Wait for the next wifi connection failure or success

        :type ssid: str
        :param ssid: ssid to control the connection
        :type timeout: int
        :param timeout: time to wait until no response arrives (optional)

        :rtype: str
        :return: "SUCCESS" for a connection success
                "FAILURE" for a connection failure
                "TIMEOUT" for no information trigged in the limited time
        """

        start_time = time.time()

        while start_time + timeout > time.time():
            connect_ssid, connect_status = self._get_wifi_connect_status()
            if ssid in connect_ssid and connect_status in SupplicantState.COMPLETED:
                self._logger.debug("Connected to network %s" % ssid)
                return "SUCCESS"

        self._logger.debug("Failure to connect to network %s" % ssid)
        return "FAILURE"

    def start_iperf_server(self, settings):
        """
        Launch iperf server on embedded side

        :type settings: dict
        :param settings:  iperf options dictionary coming from user needs, ie:
            { 'iperf':'/data/local/iperf', 'protocol':'tcp'|'udp',
              'direction':'up'|'down'|'both', 'bandwidth' : '11M'|'54M'...,
              'ftp_user': 'ssh_login' (in case 'iperf' is an ssh command) }
        :return: status, output
        """
        # ensure no iperf server is running
        self._logger.debug("Kill existing iperf server")
        self._internal_exec("killall iperf", timeout=10, raise_exception=False, use_uart=False)
        self._logger.debug("Start iperf server on port %s" % str(settings.get('port_number')))
        # Build the iperf command using ssh
        local_settings = dict(settings)
        local_settings["iperf"] = self.__ssh + "/".join([self._device.binaries_path, "iperf"])
        self._build_iperf_command(local_settings, "server")
        self._logger.info("Start IPERF server on DUT:")
        self.__proc, self.__q = run_local_command(self._iperf_cmd)

    def stop_iperf_server(self):
        serveroutput = ''
        if self.__proc is not None and self.__q is not None:
            self._logger.info("Stop IPERF server on DUT:")
            # Terminate IPERF server
            self.__proc.terminate()

            while True:
                try:
                    line = self.__q.get(timeout=.1)
                except Empty:
                    break
                else:
                    serveroutput += str(line)
            self._logger.debug("Iperf server output:\n %s" % serveroutput)
        else:
            self._logger.error("Iperf server was not started.")

        return serveroutput

    def lost_packets_iperf(self, queue):
        """
        Looks for indication of lost packet in the queue containing the output
        of an iperf client or server.
        :type queue: queue
        :param queue: queue of iperf command
        :rtype: boolean
        :return: True if lost packets are detected, False if not.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def iperf(self, settings):
        """
        Measures throughputs using IPERF

        :type settings: dictionary
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration

        :rtype: measured throughput value
        """
        exe = IperfExecutionHandler(settings, self)
        return exe.iperf()

    def ftp_xfer(self, direction, server_ip_address,
                 username, password, filename, timeout, local_path, target_throughput=None, client_ip_address=""):
        """
        Transfers a file via FTP

        :type direction: str
        :param direction: Transfer direction (UL/DL)

        :type server_ip_address: str
        :param server_ip_address: IP address of the FTP server

        :type username: str
        :param username: FTP account username

        :type password: str
        :param password: FTP account password

        :type filename: str
        :param filename: File name to be transfered

        :type timeout: integer
        :param timeout: script execution timeout

        :type local_path: str
        :param local_path: path where to read/save the FTP file (sdcard)

        :type target_throughput: float
        :param target_throughput: target throughput for data transfer in kbps

        :type client_ip_address: str
        :param client_ip_address: IP address of the DUT networking interface we want to use for FTP
        This parameter is only used on windows (it is added here for compatibility)

        :rtype: list
        :return: operation status & output log
        """
        ftp_status = False

        # Extract the file name
        ftp_filename = os.path.normpath(filename).replace("\\", "/")
        ftp_file = os.path.basename(ftp_filename)
        local_file = os.path.join(local_path, ftp_file)

        if str(direction).upper() == "UL":
            # analyse file to upload
            try:
                size_kb = self.__retrieve_size_from_filename(filename)
                self._create_file_if_needed(filename, size_kb, local_path)
            except AcsBaseException as excp:
                self._logger.info(
                    "Preliminary analysis failed : we won't be able to "
                    "create file if not already present in the device")
                self._logger.debug("Root cause : " + str(excp))
            direction_param = "-p"
        else:
            #Remove local file if already exists
            if self._phone_system.check_file_exist(local_file):
                self._logger.info("Removing local file {0} as it already exists".format(local_file))
                self._phone_system.delete(local_file)
            direction_param = "-g"

        cmd = "tftp %s -r %s -l %s %s" \
            % (str(direction_param), str(ftp_file), str(local_file), str(server_ip_address))

        status, output = self._internal_exec(cmd, raise_exception=False, use_uart=False)

        if "timeout" in output.lower():
            self._logger.error("FTP transfer: Timeout during connection")
        elif "file not found" in output.lower():
            self._logger.error("FTP transfer: File not found on server")
        elif "can't open" in output.lower():
            self._logger.error("FTP transfer: Incorrect local file")
        elif "usage: tftp" in output.lower():
            self._logger.error("FTP transfer: Incorrect tftp command")
        elif output is "":
            if str(direction).upper() == "UL":
                self._logger.info("FTP transfer: Successful transfer")
                ftp_status = True
            else:
                if self._phone_system.check_file_exist(local_file):
                    self._logger.info("FTP transfer: Successful transfer")
                    ftp_status = True
                else:
                    self._logger.error("FTP transfer: File not available")

        if ftp_status:
            return Global.SUCCESS, output
        else:
            return Global.FAILURE, output

    def _create_file_if_needed(self, filename, size_ko, dest_folder=None):
        """
        Create a file in the DUT, if it doesn't exist or if it has an incorrect
         size.

        :param filename: name of the file to create
        :type filename: str

        :param size_ko: Size in kilo-octet of the wanted file, as an integer
        strictly higher that 0
        :type size_ko: int

        :param dest_folder: Optionnal parameter if the wanted file must be
        outside of the ACS ftp directory
        :type dest_folder: str

        """
        self._logger.info(
            "Checking file existence in the device, creating it if needed")
        # check parameters
        if not filename:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Incorrect filename : cannot create file on device")
        if not isinstance(size_ko, int) or size_ko <= 0:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Incorrect file size : cannot create file on device")

        if dest_folder is None:
            dest_folder = self._device.multimedia_path
        else:
            filename = os.path.basename(filename)

        file_path = os.path.normpath(dest_folder + "/" + filename).replace("\\", "/")

        self._logger.debug("File to check : " + file_path)
        # check if file exists
        file_exists = self._phone_system.check_file_exist(file_path)

        if file_exists:
            self._logger.debug("File already existing: checking size")
            # check size
            file_size = self._phone_system.get_file_size(file_path)
            # convert file size in kilobyte
            file_size = int(file_size / 1024)

            if file_size == size_ko:
                self._logger.debug("File has the correct size. Nothing to be done")
                return
            else:
                self._logger.debug(
                    "File has an incorrect size (%d) : it will "
                    "be replaced by a new one with the wanted size."
                    % file_size)

        # create or replace file
        self._logger.debug("Create file %s of %d MB" % (file_path, size_ko))
        self._file_api.create_custom_size_user_data(file_path, size_ko, folder="")

    def __retrieve_size_from_filename(self, filename):
        """
        Try to retrieve file size from the filename.
        Filename must follow a specific format:
        [name][size (integer)][unit (k, ko, kB, m, mo, mB, g, go, gB)][extension (optional)]
        example : put500MB.zip

        :return: Size in kilo-Byte
        :rtype: int
        """
        known_units = ("ko", "kb", "mo", "mb", "go", "gb", "k", "m", "g")
        coef = {"ko": 1, "k": 1, "kb": 1,
                "mo": 1024, "m": 1024, "mb": 1024,
                "go": (1024 * 1024), "g": (1024 * 1024), "gb": (1024 * 1024)}

        # retrieve only filename
        filename = os.path.basename(filename)
        # retrieve all integers and unit from the filename
        self._logger.debug("Filename is '%s'" % str(filename))

        regex_search = re.search("^\D*(\d*)(\w).*$", str(filename))

        if regex_search is not None:
            size = str(regex_search.group(1))
            unit = str(regex_search.group(2)).lower()
            self._logger.debug("File size will be '%s'" % str(size))
            self._logger.debug("File unit will be '%s'" % str(unit))

            if unit in known_units:
                # convert size in kilobyte
                return int(int(size) * coef[unit])
            else:
                self._logger.error("Unkown unit '%s'" % str(unit))

        raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                               "Filename doesn't follow the specific format : "
                               "[name][size (integer)][unit %s]"
                               "[extension (optional)]" % str(known_units))

    def check_ipv6_consistency(self, interface, ipv6_prefix=""):
        """
        Check that the local IPV6 address is compliant with the EUI-64 format.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type ipv6_prefix: String
        :param ipv6_prefix: If you want to check the IPV6 prefix in addition to
                            the loopback one, provide this parameter.

        :rtype: bool
        :return: True if the check is good, False else.
        """
        #TODO: Implement when IPV6 client available
        self._logger.debug("launch_dhcp6_client: Not used on Linux devices")
        return True

    def disable_ip_filtering(self, phy):
        """
        Disables all IP filtering

        :type phy: String
        :param: phy the physical interface to disable the filter to.
        """
        self._logger.debug("disable_ip_filtering: Not used on Linux devices")

    def set_flight_mode(self, mode):
        """
        Sets the flight mode to off or on.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        self.__flight_mode = mode
        self._logger.debug("set_flight_mode: No Flight mode on Linux devices")

    def get_flight_mode(self):
        """
        Returns the flight mode.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        if self.__flight_mode in ("on", "1", 1):
            flight_on = 1
        elif self.__flight_mode in ("off", "0", 0):
            flight_on = 0

        self._logger.debug("get_flight_mode: No Flight mode on Linux devices")
        return flight_on

    def clean_all_data_connections(self):
        """
        Disable PDP context and remove all known Wifi networks

        :return: None
        """
        self._logger.debug("clean_all_data_connections: Not used on Linux devices")

    def simulate_phone_pin_unlock(self, pin_lock_code=""):
        """
        Simulates the entry of the PHONE pin code by the user.
        This is intended to unlock the certificate for enterprise authentication if
        it protected by one.

        :type pin_lock_code: String
        :param pin_lock_code: The code used to lock the phone
        """
        self._logger.debug("simulate_phone_pin_unlock: Not used on Linux devices")

    def launch_rdnssd_client(self, interface):
        """
        Launches the Recursive DNS Server on the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        self._logger.debug("launch_rdnssd_client: Not used on Linux devices")

    def launch_dhcp6_client(self, dhcp6_mode, interface):
        """
        Launched the ipv6 address of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type dhcp6_mode: str
        :param dhcp6_mode: stateless or stateful
        """
        #TODO: Implement when IPV6 client available
        self._logger.debug("launch_dhcp6_client: Not used on Linux devices")

    def start_wifi_connection_log(self):
        """
        Start logging information about Wifi networks connection
        """
        self._logger.debug("get_wifi_connection_status_log: Not used on Linux devices")

    def set_wifi_sleep_policy(self, policy):
        """
        Set the wifi sleep policy

        :type policy: WIFI_SLEEP_POLICY enum
        :param policy: policy to set

        """
        self._logger.debug("set_wifi_sleep_policy: Not used on Linux devices")

    def get_wifi_hotspot_status(self):
        """
        Get the status of the Wifi hotspot feature.

        :rtype: int
        :return: 1 if enabled, 0 if disabled
        """
        status, output = self._internal_exec(self.__get_hostapd_status, raise_exception=False)
        if "Active: active (running)" in str(output):
            self._logger.debug("Hostapd mode is ON")
            power = 1
        elif "Active: inactive (dead)" in str(output) or "stopped hostap daemon service." in str(output).lower():
            self._logger.debug("Hostapd mode is OFF")
            power = 0
        else:
            msg = "Unable to get hostap daemon status ({0})".format(output)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, msg)

        return power

    @need('modem', False)
    def deactivate_pdp_context(self, interface=None, check=True):
        """
        Deactivates a Packet Data Protocol (PDP) context using its interface

        .. note:: interface parameter not used on Android
        :type interface: str :param in
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context deactivation is checked (no check for GSM)

        :return: None
        """
        self._logger.debug("deactivate_pdp_context: Not used on Linux devices")

    def wifi_menu_settings(self, displayed=True):
        """
        Enter WiFi menu setting or exit from WiFi menu settings
        This UEcmd is useful for WiFi policy tests

        :type displayed: bool
        :param displayed: True to enter into WiFi menu setting\
                            and False to exit (goes to idle screen)
        """
        self._logger.debug("set_display_brightness: No display on Linux devices")

    def start_iperf_client(self, settings):
        protocol = settings.get('protocol', 'tcp')
        port_number = settings.get('port_number')
        duration = settings.get('duration', '10')
        direction = settings.get('direction', 'both')
        msg = "IPERF %s " % protocol.upper()
        if direction == 'both':
            msg += "UL & DL"
        else:
            msg += direction.upper()
        msg += " measurement on port %s for %s seconds" % (str(port_number), str(duration))
        self._logger.info(msg)
        # Build the iperf command
        self._build_iperf_command(settings, "client")
        self._logger.debug("Run command : " + self._iperf_cmd)
        output = ""
        try:
            status, output = self._ssh_api.ssh_run_cmd(self._iperf_cmd)
        except:
            # Disconnection
            self._logger.error("iperf client disconnected: \n" + output)
            raise

        self._logger.debug("iperf client output:\n" + output)

        return output
