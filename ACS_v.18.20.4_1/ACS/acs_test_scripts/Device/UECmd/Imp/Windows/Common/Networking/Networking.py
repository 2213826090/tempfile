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
:summary: This file implements Networking UECmds
:since: 04 Jan 2012
:author: sfusilie, dgonza4x
"""
from multiprocessing import Queue
from threading import Thread

import time
import os
import re
from Queue import Empty

from UtilitiesFWK.Utilities import Global, is_number, str_to_bool
from acs_test_scripts.Utilities.IPerfUtilities import \
    IperfOptions, IperfExecutionHandler, search_for_mandatory_keyword_arguments
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from acs_test_scripts.Device.UECmd.UECmdTypes import Measure, AUTO_CONNECT_STATE, PreferredNetwork
from acs_test_scripts.Device.UECmd.Interface.Networking.INetworking import INetworking
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from acs_test_scripts.Utilities.IPerfUtilities import Iperf


class iperf_process(Thread):
    """
    Class that handles iperf process.
    On android DUT iperf is directly called from adb shell
    so by stopping a process, iperf is stopped
    On windows DUT iperf is called from an Uecmd and we need to call another Uecmd to stop it
    so we need to create a special class to handle iperf process stop
    """
    def __init__(self, networking_api, target=None, args=()):
        self._networking_api = networking_api
        Thread.__init__(self, target=target, args=args)

    def terminate(self):
        # Send stopIperf Uecmd and wait for its completion
        self._networking_api.stop_iperf()
        self.join(250)


class Networking(Base, INetworking, Iperf):

    """
    Class that handles all networking operations.
    """

    _apn_name = None
    last_set_data_class = 0

    """
    Conversion table between generic preferred network type and windows preferred network type
    """
    NETWORK_TYPE_CONVERSION_TABLE = \
    {  #      ANDROID                        WINDOWS
        "3G_PREF"      : 0x0000001F,  # 0: "WCDMA_PREF",                   0x0000001F: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS_EDGE_GPRS"
        "2G_ONLY"      : 0x00000003,  # 1: "GSM_ONLY",                     0x00000003: "DATA_CLASS_EDGE_GPRS"
        "3G_ONLY"      : 0x0000001C,  # 2: "WCDMA_ONLY",                   0x0000001C: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS
        "2G_3G"        : 0x0000001F,  # 3: "GSM_UMTS",                     0x0000001F: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS_EDGE_GPRS"
        "CDMA_PREF"    : 0x003F0000,  # 4: "CDMA",                         0x003F0000: "DATA_CLASS_1XRTT_3XRTT_1XEVDO_REVA_REVB_EVDV"
        "CDMA_ONLY"    : 0x00110000,  # 5: "CDMA_NO_EVDO"                  0x00110000: "DATA_CLASS_1XRTT_3XRTT"
        "EVDO_ONLY"    : 0x002D0000,  # 6: "EVDO_NO_CDMA"                  0x002D0000: "DATA_CLASS_1XEVDO_REVA_REVB_EVDV"
        "GLOBAL"       : 0x03F0003E,  # 7: "GLOBAL",                       0x03F0003E: "DATA_CLASS_1XRTT_EVDO_HSUPA_HSDPA_UMTS_EDGE_GPRS"
        "4G_PREF_US"   : 0x03F00020,  # 8: "LTE_CDMA_EVDO",                0x03F00020: "DATA_CLASS_LTE_1XRTT_3XRTT_1XEVDO_REVA_REVB_EVDV "
        "4G_PREF"      : 0x0000003F,  # 9: "LTE_GSM_WCDMA",                0x0000003F: "DATA_CLASS_LTE_HSUPA_HSDPA_UMTS_EDGE_GPRS"
        "WORLD_MODE"   : 0x03F0003F,  # 10: "LTE_CMDA_EVDO_GSM_WCDMA"      0x03F0003F: "DATA_CLASS_LTE_1XRTT_3XRTT_1XEVDO_REVA_REVB_EVDV_HSUPA_HSDPA_UMTS_EDGE_GPRS"
        "4G_ONLY"      : 0x00000020  # 11: "LTE_ONLY"                      0x00000020: "DATA_CLASS_LTE"
    }

# Frequently used network types
#    0x00000000: "DATA_CLASS_NONE",                              # 0
#    0x00000001: "DATA_CLASS_GPRS",                              # 1
#    0x00000002: "DATA_CLASS_EDGE",                              # 2
#    0x00000003: "DATA_CLASS_EDGE_GPRS",                         # 3 = "2G_ONLY"
#    0x00000004: "DATA_CLASS_UMTS",                              # 4
#    0x00000007: "DATA_CLASS_UMTS_EDGE_GPRS",                    # 7
#    0x00000008: "DATA_CLASS_HSDPA",                             # 8
#    0x00000010: "DATA_CLASS_HSUPA",                             # 16
#    0x00000018: "DATA_CLASS_HSPA",                              # 24
#    0x0000001F: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS_EDGE_GPRS",   # 31 = "3G_PREF"
#    0x00000020: "DATA_CLASS_LTE",                               # 32 = "4G_ONLY"
#    0x0000003F: "DATA_CLASS_LTE_HSUPA_HSDPA_UMTS_EDGE_GPRS",    # 63 = "4G_PREF"

    def InitializePreferredNetworkClass(self):
        PreferredNetwork.GSM_PREFERRED_NETWORKS = ("3G_PREF", "2G_ONLY", "2G_3G", "GLOBAL", "4G_PREF", "WORLD_MODE", "WIN_0x00000001", "WIN_0x00000002", "WIN_0x00000007")
        PreferredNetwork.CDMA_PREFERRED_NETWORKS = ()
        PreferredNetwork.EVDO_PREFERRED_NETWORKS = ()
        PreferredNetwork.WCDMA_PREFERRED_NETWORKS = ("3G_PREF", "3G_ONLY", "2G_3G", "GLOBAL", "4G_PREF", "WORLD_MODE", "WIN_0x00000001", "WIN_0x00000002", "WIN_0x00000004", "WIN_0x00000007")
        PreferredNetwork.HSPA_PREFERRED_NETWORKS = ("3G_PREF", "3G_ONLY", "2G_3G", "GLOBAL", "4G_PREF", "WORLD_MODE", "WIN_0x00000001", "WIN_0x00000002", "WIN_0x00000004", "WIN_0x00000007", "WIN_0x00000008", "WIN_0x00000010", "WIN_0x00000018")
        PreferredNetwork.LTE_PREFERRED_NETWORKS = ("4G_PREF_US", "4G_PREF", "WORLD_MODE", "4G_ONLY")

        PreferredNetwork.GSM_RAT = ("GPRS", "EGPRS")
        PreferredNetwork.CDMA_RAT = ()
        PreferredNetwork.EVDO_RAT = ()
        PreferredNetwork.WCDMA_RAT = ("UMTS", "HSDPA", "HSUPA", "HSPA")
        PreferredNetwork.LTE_RAT = ("LTE")

        PreferredNetwork.GSM_ONLY = "2G_ONLY"
        PreferredNetwork.WCDMA_ONLY = "3G_ONLY"
        PreferredNetwork.LTE_ONLY = "4G_ONLY"
        PreferredNetwork.WCDMA_PREF = "3G_PREF"

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        INetworking.__init__(self, device)

        self.InitializePreferredNetworkClass()

        self._module_name = "Intel.Acs.TestFmk.Networking"
        self._class_name = "Intel.Acs.TestFmk.Networking.NetworkingActivity"

        self._phone_system = device.get_uecmd("PhoneSystem")
        self.__file_uecmds = device.get_uecmd("File")
        self._q = Queue()
        # todo remove this parameter when set_wifi_frequency_band and get_wifi_frequency_band are implemented
        self.__mock_freq_band = ""
        self._flight_mode = 0

    def get_translated_network_type(self, network_type):
        """
        Return the string representation of the network type.

        :type network_type: int
        :param network_type: can be the index of NETWORK_TYPE_CONVERSION_TABLE, or WIN_0x12345678 in case it is not defined.

        :return: network_type as a string
        """
        res = ""
        if network_type in self.NETWORK_TYPE_CONVERSION_TABLE.values():
            res = self.NETWORK_TYPE_CONVERSION_TABLE.keys()[self.NETWORK_TYPE_CONVERSION_TABLE.values().index(network_type)]
        else:
            res = "WIN_0x%08X" % network_type
        return res

    def get_untranslated_network_type(self, network_type):
        """
        Return the integer representation of the network type.

        :type network_type: str
        :param network_type: can be the index of NETWORK_TYPE_CONVERSION_TABLE, or WIN_0x12345678 in case it is not defined.

        :return: network_type as a int
        """
        res = -1
        if network_type in self.NETWORK_TYPE_CONVERSION_TABLE.keys():
            res = self.NETWORK_TYPE_CONVERSION_TABLE[network_type]
        elif len(network_type) > 6:
            try:
                res = int(network_type[6:], 16)
            except:
                res = -1
        else:
            res = -1
        return res

    def is_preferred_network_type_valid(self, network_type):
        """
        Checks whether the network type is valid.

        :type network_type: str
        :param network_type: can be one of NETWORK_TYPE_CONVERSION_TABLE dictionary elements, or defined as "WIN_0x12345678".

        :rtype: bool
        :return: True for valid or False for invalid network type
        """
        res = False
        if self.get_untranslated_network_type(network_type) >= 0:
            res = True
        return res

    @need('wifi', False)
    def set_wifi_power(self, mode):
        """
        Sets the WIFI power to off or on.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        wifi_adapter_name = self._device.get_wifi_adapter_name()
        function = "TurnOnOffWiFiFromNetworkAdapter"
        # Get the method and class name of the UEcommand on the embedded side
        args = "action="
        module_name, class_name = self._get_module_and_class_names("Networking")
        if mode == 'on' or mode == "1" or mode == 1:
            args += "TurnOn"
        elif mode == 'off' or mode == "0" or mode == 0:
            args += "TurnOff"
        else:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "<mode> in set_wifi_power(%s)", mode)
        args += " network_apdater_name=" + str(wifi_adapter_name)

        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, function, args, 60)

    @need('wifi')
    def request_wifi_scan(self):
        """
        Triggers a Wifi scan.

        :return: None
        """
        scan_timeout = 10
        fct_name = "RequestScan"
        args = "scan_timeout=" + str(scan_timeout)
        module_name, class_name = self._get_module_and_class_names("WifiConnectivity")
        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name,
            args)
        values = output["values"]
        # failure cases are automatically handled.
        # if success, write the time for scan to be finished
        if "scan_complete" in values.keys():
            scan_complete = values["scan_complete"]
            if scan_complete and "time_to_complete" in values.keys():
                self._logger.debug("Scan completed in approximately %s seconds", values["time_to_complete"])

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
        if technology not in ("wifi", "bluetooth", "all"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "%s not supported !" % technology)

        ssids = []
        if technology == "wifi":
            module_name, class_name = self._get_module_and_class_names("WifiConnectivity")
            output = self._internal_uecmd_exec(
                module_name,
                class_name,
                "EnumerateWifiNetworks")

            ssids = output["values"]["networks"]
        else:
            self._logger.warning("Scan not supported for %s" % technology)

        return ssids

    @need('wifi')
    def wifi_menu_settings(self, displayed=True):
        """
        Enter WiFi menu setting or exit from WiFi menu settings
        This UEcmd is useful for WiFi policy tests

        :type displayed: bool
        :param displayed: True to enter into WiFi menu setting\
                            and False to exit (goes to idle screen)
        """
        self._logger.warning("[NOT IMPLEMENTED] Networking.wifi_menu_settings")

    def open_web_browser(self, website_url, browser_type="native", timeout=None, skip_eula=False):
        """
        Open the Web Browser on the web page
        passed as parameter.

        :type website_url: str
        :param website_url: URL to open

        :type browser_type: str
        :param browser_type: "native" will open the default browser,
            other type can be added depending on the os

        :type timeout: int
        :param timeout: timeout to open the page

        :type skip_eula: boolean
        :param skip_eula: skip EULA on 1st start

        :rtype: tuple
        :return: operation status & output log
        """

        error_code = Global.SUCCESS
        error_msg = "No errors"

        # Verification on the value of the browser_type parameter
        if browser_type not in ("native", "acs"):
            error_code = Global.FAILURE
            error_msg = "browser type %s is not valid. Must be either native \
                        or acs" % browser_type
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid !")

        # Verification on the value of the timeout parameter
        if timeout is None:
            timeout = self._uecmd_default_timeout

        if not isinstance(timeout, int) or timeout <= 0:
            error_code = Global.FAILURE
            error_msg = "Parameter timeout must be integer superior to 0. Here timeout=%s" % timeout
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Construct the command necessary to launch the UEcmd on embedded side
        # with the method name and the necessary arguments
        function = "Open_Web_Browser"
        args = "website_url=%s browser_type=%s \
                    timeout=%s" % (website_url, browser_type, timeout)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, function, args, timeout)

        return error_code, error_msg

    def close_web_browser(self, browser_type="native"):
        """
        Close the Web Browser.

        :type browser_type: str
        :param browser_type: "native" will open the default browser,
            other type can be added depending on the os

        :return: None
        """
        if browser_type not in ("native", "acs"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid !")

        # Construct the command necessary to launch the UEcmd on embedded side
        # with the method name and the necessary arguments
        function = "Close_Web_Browser"
        args = "browser_type=%s" % browser_type

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, function, args)

    def get_flight_mode(self):
        """
        Returns the flight mode.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        method_name = "GetAirplaneModeByUI"
        # method_name = "get_airplane_mode_state"
        args = ''
        module_name, class_name = self._get_module_and_class_names("UIAction")
        # module_name, class_name = self._get_module_and_class_names("UIActionCenter")
        # Launch the UEcmd on the embedded side
        time.sleep(5)
        output = self._internal_uecmd_exec(module_name, class_name, method_name, args, 120)
        result = output['airplane_mode_state']
        if result == 'on':
            flight_mode = 1
        elif result == 'off':
            flight_mode = 0
        else:
            flight_mode = 0
            self._logger.error("Fail to get_flight_mode by default we considerate flight mode at off")
        return flight_mode

    def set_flight_mode(self, mode):
        """
        Sets the flight mode to off or on.
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        args = ""
        if mode in ("on", "1", 1):
            args = "action=TurnOn"
            # method_name = "enable_airplane_mode"
        else:
            args = "action=TurnOff"
            # method_name = "disable_airplane_mode"
        method_name = "TurnOnOffAirplaneModeByUI"
        module_name, class_name = self._get_module_and_class_names("UIAction")
        # module_name, class_name = self._get_module_and_class_names("UIActionCenter")
        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, method_name, args, 120)

    @need('wifi')
    def clean_all_data_connections(self):
        """
        Disable PDP context and remove all known Wifi networks

        :return: None
        """
        self._logger.info("Clean all data connections")
        self.wifi_remove_config("all")

    # @need('modem')
    def simulate_phone_pin_unlock(self, pin_lock_code=""):
        """
        Simulates the entry of the PHONE pin code by the user.
        This is intended to unlock the certificate for enterprise authentication if
        it protected by one.

        :type pin_lock_code: str
        :param pin_lock_code: The code used to lock the phone
        """
        self._logger.warning("[NOT IMPLEMENTED] Networking.simulate_phone_pin_unlock")
        # TODO : implements it

    @need('wifi')
    def set_wificonfiguration(self, ssid, pass_phrase, security,
                              ip_method="dhcp", address="", netmask="",
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
        module_name, class_name = self._get_module_and_class_names("WifiConnectivity")

        # configure IP properties
        fct_name = "SetIPV4Properties"
        args = "mode=%s ip=%s subnet_mask=%s gateway=%s dns1=%s dns2=%s" % \
            (ip_method, address, netmask, gateway, dns1, dns2)
        self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name,
            args)

        # setup the wireless network
        security_upper = self._get_wifi_security_type(security)
        fct_name = "SetWifiConfiguration"
        args = "auto_connect=false security=%s ssid=%s" % (str(security_upper).lower(), ssid)
        if pass_phrase:
            args += " passphrase=%s" % (pass_phrase)

        self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name,
            args)

    # @need('wifi')
    def set_wifi_frequency_band(self, freq_band, silent_mode=False, interface="wlan0"):
        """
        Set the Wifi Frequency Band

        :type freq_band: str
        :param freq_band: Frequency Band to set (auto, 2.4GHz, 5GHz)

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception
                            if the device does not support this method

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        self._logger.warning("[NOT IMPLEMENTED] Networking.set_wifi_frequency_band")
        # TODO : implements it ! (and remove stubbed code)
        self.__mock_freq_band = freq_band

    # @need('wifi')
    def set_regulatorydomain(self, regulatory_domain, interface=""):
        """
        Set the Wifi Regulatory Domain

        :type regulatory_domain: str
        :param regulatory_domain: the regulatory domain to set (FR, GB, US...)
        """
        self._logger.warning("[NOT IMPLEMENTED] Networking.set_regulatorydomain")
        # TODO : implements it

    # @need('wifi')
    def get_regulatorydomain(self):
        """
        Get the Wifi Regulatory Domain

        :rtype: str
        :return: the regulatory domain (FR, GB, US... or "none")
        """
        self._logger.warning("[NOT IMPLEMENTED] Networking.get_regulatorydomain")
        # TODO : implements it

    @need('wifi', False, 0)
    def get_wifi_power_status(self):
        """
        Gets the WIFI power.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        wifi_adapter_name = self._device.get_wifi_adapter_name()
        function = "GetWifiPowerStatus"
        args = "network_apdater_name=" + str(wifi_adapter_name)
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args, 120)
        self._logger.info("Wifi power status: " + output["wifi_power_status"])

        return output["wifi_power_status"]

    @need('wifi')
    def set_wifi_power_saving_mode(self, mode, interface=None):
        """
        Sets the power saving mode of the given wlan interface to the given value.
        This method calls the internal C{_set_wifi_power_saving_mode} method
        and adds some additional checks and log messages.

        :attention: 'interface' parameter isn't used on Windows.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :type interface: str
        :param interface: the wlan interface name

        :return: None
        """
        # Initialize variables
        if mode in ("on", "1", 1):
            mode = "1"
        elif mode in ("off", "0", 0):
            mode = "0"
        else:
            output = "%s is not in known mode" % mode
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, output)
        # Write a log message
        self._logger.info("set_wifi_power_saving_mode to '%s'" % mode)
        # Do the job
        self._set_wifi_power_saving_mode(mode, interface)

    @need('wifi')
    def _set_wifi_power_saving_mode(self, mode, interface=None):  # pylint: disable=W0613
        """
        Method that actually sets the power saving mode of the given
        wlan interface to the given value.
        This method is for internal use (not additional 'info' log).

        @attention: 'interface' parameter isn't used on Windows.

        :type mode: str
        :param mode: the requested mode. Possible values:
            - "0": off
            - "1": on

        :type interface: str
        :param interface: the wlan interface name

        :return: None
        """
        if mode not in ("0", "1"):
            message = "Invalid parameter value for 'mode': %s" % mode
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Set the wifi power saving mode
        function = "SetWifiPowerSavingMode"
        args = "mode=%s" % (mode)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, function, args)

    @need('wifi')
    def _get_wifi_chipset_manufacturer(self):
        """
        Get Wifi chipset manufacturer: CHIPSET_BROADCOM or CHIPSET_TI
        List of possible values are attributes of this class

        :rtype: str
        :return: The Wifi chipset used on the PHONE
        """
        if self._wifi_chipset_manufacturer is None:
            function = "GetWifiChipsetManufacturer"

            # Get the method and class name of the UEcommand on the embedded side
            module_name, class_name = self._get_module_and_class_names()

            # Launch the UEcmd on the embedded side
            output = self._internal_uecmd_exec(module_name, class_name, function)
            out = output["values"]["manufacturer"]

            if re.search(r'^wl12xx', out, re.MULTILINE) is not None:
                self._logger.debug("Wifi Chipset TI detected")
                self._wifi_chipset_manufacturer = self.CHIPSET_TI
            else:
                if re.search(r'^bcm', out, re.MULTILINE) is not None:
                    self._logger.debug("Wifi Chipset Broadcom detected")
                else:
                    self._logger.debug("Unable to detect Wifi Chipset. Adopting Broadcom behavior")
                self._wifi_chipset_manufacturer = self.CHIPSET_BROADCOM

        return self._wifi_chipset_manufacturer

    @need('wifi')
    def reset_mobile_broadband_mode(self, mode):
        """
        Sets MobileBroadband to off or on.
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        modem_api = self._device.get_uecmd("Modem", True)
        self._logger.info("Mobile Broadband on/off")
        current_mode = str(modem_api.get_modem_power_status())

        if mode in ("on", "1", 1):  # mode to be set is "on"
            if current_mode in ("on", "1", 1):
                self._logger.info("Mobile Broadband currently is ON")
                modem_api.set_modem_power("off")
                time.sleep(5)
                modem_api.set_modem_power("on")
            elif current_mode in ("off", "0", 0):
                self._logger.info("Mobile Broadband currently is OFF")
                modem_api.set_modem_power("on")
            else:
                error_msg = "Mobile Broadband : Parameter mode %s is not valid" % mode
                self._logger.error(error_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        else:  # mode to be set is "off"
            if current_mode in ("on", "1", 1):
                self._logger.info("Mobile Broadband currently is ON")
                modem_api.set_modem_power("off")
            elif current_mode in ("off", "0", 0):
                self._logger.info("Mobile Broadband curretly is OFF:")
            else:
                error_msg = "Mobile Broadband : Parameter mode %s is not valid" % mode
                self._logger.error(error_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

    @need('modem', False)
    def activate_pdp_context(self, apn_name=None, check=True):
        """
        Activates a Packet Data Protocol (PDP) context using its interface

        :note: apn_name parameter not used on Windows
        :type interface: str
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context activation is checked (no check for GSM)

        :return: None
        """
        # turn on modem if it is off
        modem_api = self._device.get_uecmd("Modem", True)
        if modem_api.get_modem_power_status() == 0:
            self._logger.info("modem is off, can not activate pdp")
            return

        # check register status
        time.sleep(5)
        reg_state = modem_api.get_network_registration_status()
        if reg_state in ["none", "unregistered", "denied", "searching"]:
            self._logger.info("Modem is not registered to network, could not activate PDP connection")
            return

        self._logger.info("Checking PDP Context Status")
        time.sleep(5)
        pdp_context_status = self._get_pdp_context_status()
        self._logger.info("Current PDP Context Status is %s" % pdp_context_status)

        if (pdp_context_status in ("0", "1")):
            if (pdp_context_status == "0"):
                self._logger.info("PDP Context Enabled but not yet connected")
            elif (pdp_context_status == "1"):
                self._logger.info("PDP Context Enabled and already connected")
                return
        elif (pdp_context_status == "2"):
            # for auto-pdp-activation, this state means "Activating",
            self._logger.info("PDP Context is activating...., wait until activated")
            while True:
                time.sleep(5)
                pdp_context_status = self._get_pdp_context_status()
                if pdp_context_status == "1":
                    return
        else:
            function = "ActivatePDPContext"
            args = "apn_name=%s time_out=%s" % (self._apn_name, str(self._uecmd_default_timeout * 1000))
            # Get the method and class name of the UEcommand on the embedded side
            module_name = "Intel.Acs.TestFmk.MBConnectivity"
            class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"

            # Launch the UEcmd on the embedded side
            self._internal_uecmd_exec(module_name, class_name, function, args, 60)

        # Check activation
        if check:
            time_count = 0
            connection_state = False
            while ((not connection_state) and
                   time_count <= (self._uecmd_default_timeout)):

                pdp_context_status = self._get_pdp_context_status()

                if (pdp_context_status == "1"):
                    connection_state = True
                    break

                time.sleep(1)
                time_count += 1

            if (not connection_state):
                msg = "Active PDP context timeout !"
                self._logger.error(msg)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)

    @need('modem', False)
    def reactivate_pdp_context(self, apn_name, check=True):
        """
        Disconnect PDP then reconnect it

        :note: apn_name parameter not used on Windows
        :type interface: str
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context activation is checked (no check for GSM)

        :return: None
        """
        self.deactivate_pdp_context(apn_name, check)
        self.activate_pdp_context(apn_name, check)

    @need('modem', False)
    def deactivate_pdp_context(self, apn_name=None, check=True):
        """
        Deactivates a Packet Data Protocol (PDP) context using its interface

        :note: apn_name parameter not used on Windows
        :type interface: str
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context deactivation is checked (no check for GSM)

        :return: None
        """

        # check modem radio status
        modem_api = self._device.get_uecmd("Modem", True)
        if modem_api.get_modem_power_status() == 0:
            self._logger.info("Modem is power off, no deactivation could happen")
            return

        # check register status
        reg_state = modem_api.get_network_registration_status()
        time.sleep(5)
        if reg_state in ["none", "unregistered", "denied", "searching"]:
            self._logger.info("Modem is not registered to network, no deactivation could happen")
            return

        self._logger.info("Checking PDP Context Status")

        pdp_context_status = self._get_pdp_context_status()
        if (pdp_context_status == "3" or pdp_context_status == "4"):
            self._logger.info("PDP Context already disabled")
            return

        elif (pdp_context_status in ("0", "1", "2")):
            if (pdp_context_status == "0"):
                self._logger.info("PDP Context enabled but not connected")
            if (pdp_context_status == "1"):
                self._logger.info("PDP Context enabled and connected")
            else:
                self._logger.info("PDP Context is activating")

            self._logger.info("Disabling PDP Context ...")

            function = "DeactivatePDPContext"
            args = "time_out=%s" % ("60000")

            # Get the method and class name of the UEcommand on the embedded side
            module_name = "Intel.Acs.TestFmk.MBConnectivity"
            class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"

            time.sleep(5)

            # Launch the UEcmd on the embedded side
            self._internal_uecmd_exec(module_name, class_name, function, args, 60)

        # Checking deactivation
        if check:
            time_count = 0
            connection_state = False
            while ((not connection_state) and
                   time_count <= (self._uecmd_default_timeout)):

                pdp_context_status = self._get_pdp_context_status()

                if (pdp_context_status == "3"):
                    connection_state = True
                    break
                time.sleep(1)
                time_count += 1

            if (not connection_state):
                error_msg = "Deactivate pdp context timeout !"
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, error_msg)

    @need('modem')
    def _get_pdp_context_status(self):
        """
        Returns the PDP Context status on the DUT

        :rtype: str
        0: enable but not connected, 1: enabled and connected, 2:  disabled, need to activated again
        :return: the pdp context status
        """
        function = "GetPDPContextStatus"

        module_name = "Intel.Acs.TestFmk.MBConnectivity"
        class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"
        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        return str(output["values"]["pdp_status"])

    # @need('modem')
    def set_apn(self, interface=None, apn=None, user=None, password=None, protocol=None, mmsc=None, apn_type=None, set_default=True, clear_apn=True):
        """
        Set the APN for a PDP Context using its interface

        :type interface: str
        :param interface: Data Cellular interface

        :type apn: str
        :param apn: Cellular Access Point Name

        :type user: str
        :param user: The APN user

        :type password: str
        :param password: The APN user

        :type protocol: str
        :param protocol: The APN protocol ("IP", "IPV6", "IPV4V6")

        :type mmsc: str
        :param mmsc: The APN MMSC
        (ex : http://10.102.161.47:8080/8960=8960)

        :type apn_type: str
        :param apn_type: The APN Type (ex : default,mms)

        :type set_default: bool
        :param set_default: Boolean determining if new APN is set as default APN

        :type clear_apn: bool
        :param clear_apn: Boolean determining if existing APN(s) is/are keep or remove

        :return: None
        """
        # for Windows tablet, this function is just used to keep the apn name for the successive call to activate_pdp_context.
        # TODO: Critical UEcmd
        self._apn_name = "None"

    def clear_apn(self):
        """
        Clear the current APN configuration on the DUT
        :return: None
        """
        self._logger.warning("clear_apn is stubbed for now on Windows")

    # @modem('modem')
    def set_roaming_mode(self, mode):
        """
        Sets the roaming mode to off or on.

        @type mode: string
        @param mode: can be 'on' to enable
                            'off' to disable

        @return: None
        """
        # TODO: Critical UEcmd
        self._logger.warning("Set roaming mode is stubbed for now on Windows")

    # need('modem')
    def get_roaming_mode(self):
        """
        Returns the roaming mode.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        method = "GetNetworkRegistrationStatus"

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, method)

        register_state = int(output["values"]["register_state"])

        # TODO: need to add if statement to check if roaming is register_state returned by method or not

    @need('wifi or modem or ethernet')
    def check_ip_protocol(self, protocol):
        """
        Check IP protocol used

        :type protocol: str
        :param silent_mode: IP protocol selected on the DUT for data registration

        :rtype: str
        :return: The message containing the IP address and the protocol
        """
        network_interface = self._device.get_cellular_network_interface()
        # If the protocol is IPV6 only
        # We check the device has IPV6 but not IPV4 address
        if protocol == "IPV6":
            ipv6_address = self.get_interface_ipv6_address(network_interface)
            try:
                ipv4_address = self.get_interface_ipv4_address(network_interface)
            except AcsBaseException:
                self._logger.info("The device doesn't have an IPV4 address")
            else:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "The device get an IPV4 address on IPV6 only mode %s " % ipv4_address)
            msg = "The device IPV6 address : %s" % ipv6_address

        elif protocol == "IPV4V6":
            try:
                ipv6_address = self.get_interface_ipv6_address(network_interface)
                ipv4_address = self.get_interface_ipv4_address(network_interface)
            except AcsBaseException:
                self._logger.info("The device doesn't have an IPV4 address %s nor an IPV6 Address %s "
                                  % ipv4_address, ipv6_address)
            msg = "The device IPV4 address : %s and IPV6 address : %s" % (ipv4_address, ipv6_address)

        elif protocol == "IPV4":
            ipv4_address = self.get_interface_ipv4_address(network_interface)
            # Windows defines a local IPV6 address even if we are in IPV4 only mode
            # So do not check it
            msg = "The device IPV4 address : %s" % ipv4_address
        else:
            error_msg = "Unknown PROTOCOL type, PROTOCOL type should be IPV4, IPV6, IPV4V6"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        return msg

    @need('wifi or modem or ethernet')
    def get_interface_ipv6_address(self, interface):
        """
        Returns the 1st ipv6 address of the given interface.
        :note: This function returns only the 1st address of the IPv6 link,
        get_interface_ipv6_all_address will return all the addresses.

        :rtype: str
        :return: interface ip address
        """
        function = "Check_IP_Add"
        args = "InterfaceName=%s" % (interface)
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)
        ip = output["values"]["ip6_address"]

        if ip is None:
            msg = "Could not get IPV6 address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        return ip

    @need('wifi or modem or ethernet')
    def get_interface_ipv4_address(self, interface):
        """
        Returns the ipv4 address of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: str
        :return: interface ip address
        """
        function = "Check_IP_Add"
        args = "InterfaceName=%s" % (interface)
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)

        ip = str(output["values"]["ip4_address"])

        if ip is None:
            msg = "Could not get IPV4 address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        return ip

    @need('wifi or ethernet or modem')
    def get_interface_ipv4_all_address(self):
        """
        Returns ipv4 addresses for all active network interface.

        :rtype: dict
        :return: {active interface name: ip address}
        """
        function = "Get_All_IPV4_address"
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        if_dict = output["values"]["interfaces"]

        return if_dict

    def get_interface_ipv6_scopelink_address(self, interface=""):
        """
        Returns the scope link ipv6 address of the given interface.
        .. note:: This function returns only the 1st address of the IPv6 link,

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: str
        :return: interface ip address
        """

        return self.get_interface_ipv6_address(self._device.get_cellular_network_interface())

    def check_no_ip_address(self):
        """
        Check that no IP address is given to the DUT (if data is not activated)
        """
        # TODO - these codes don't work, so temporarily commented out. currently dut return both IPV4 and IPV6 ip addresses without PDP activated
        network_interface = self._device.get_cellular_network_interface()

        function = "Check_IP_Add"
        args = "InterfaceName=%s" % (network_interface)  # changed from "interface" to "network_interface"
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)

        # We check the device does not have any IPV6 and IPV4 addresses

        ipv6_address = output["values"]["ip6_address"]
        ipv4_address = output["values"]["ip4_address"]

        print "ipv4 address is " + ipv4_address
        print "ipv6 address is " + ipv6_address

        # We check that no IPV6 has been retrieved, else we raise an exception
        if (ipv6_address is None or ipv6_address == ""):
            self._logger.info("The device doesn't have any IPV6 address")
        else:
            error_msg = "DUT has IPV6 address !"
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, error_msg)

        # We check that no IPV4 has been retrieved, else we raise an exception
        if (ipv4_address is None or ipv4_address == ""):
            self._logger.info("The device doesn't have any IPV4 address")

        else:
            error_msg = "DUT has an IPV4 address ! "
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, error_msg)

    def get_ftp_xfer_status(self):
        """
        get the xfer status.

        :rtype: str
        :return: xfer status
                - "connecting"
                - "connected"
                - "transferring"
                - "disconnected"
                - "notrunning"
        """
        self._logger.info("Get FTP transfer status...")

        function = "Get_FTP_Xfer_Status"
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        return output["values"]["status"]

    @need('wifi or modem or ethernet')
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

        :rtype: list
        :return: operation status & output log
        """
        if local_path is None or local_path == "":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "local FTP directory path is not defined")
        local_dir = os.path.dirname(local_path)
        if self._phone_system.check_directory_exist_from_shell(local_dir) is False:
            self._logger.info("Local path is not valid, create it")
            self._phone_system.create_directory_from_shell(local_dir)

        # Preliminary analysis : create file to upload if not
        # already present in the device (IF POSSIBLE)
        if str(direction).upper() == "UL":
            # analyse file to upload
            try:
                size_kb = self.__file_uecmds.retrieve_size_from_filename(filename)
                self.__file_uecmds.create_file_if_needed(filename, size_kb, local_path)
            except AcsBaseException as excp:
                self._logger.info(
                    "Preliminary analysis failed : we won't be able to "
                    "create file if not already present in the device")
                self._logger.debug("Root cause : " + str(excp))

        # Extract the ftp path and ftp file
        filename = os.path.normpath(filename).replace("\\", "/")
        local_file = os.path.join(local_path, os.path.basename(filename)).replace("\\", "/")

        # Start ftp transfer
        function = "FTP_xfer"
        args = "direction=%s server_ip_address=%s username=%s password=%s remote_filename=%s local_filename=%s client_ip_address=%s" % \
               (direction, server_ip_address, username, password, filename, local_file, client_ip_address)
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        try:
            # Launch the UEcmd on the embedded side
            ftp_result = self._internal_uecmd_exec(module_name, class_name, function, args, timeout=timeout)
        except AcsBaseException as error:
            try:
                self.stop_ftp_xfer(None, True)
            except AcsBaseException as stop_error:
                # Log warning message in case stop ftp fails
                warning_msg = "Stop FTP transfer failed ! (%s)" % stop_error.get_error_message()
                self._logger.warning(warning_msg)
            # Return FTP transfer error when timeout on uecommand result is reached
            if error.get_error_message().find(self.UECMD_TIMEOUT_RESULT_MSG) != -1:
                error_msg = "FTP Transfer timeout !"
                self._logger.info("FTP Transfer timeout, stopping remaining transfer")
                # Raise FTP transfer timeout
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error_msg)
            else:
                raise error

        throughput = float(ftp_result["values"].get("throughput"))
        log = ftp_result["values"].get("logs")
        self._logger.debug("FTP client output log: \n %s" % log)

        # If a target throughput has been given, check if measured throughput is greater or equal than the target value
        if target_throughput is not None:
            output = str(direction) + " file: " + str(filename) + " successfully"
            output += " - target throughput: " + str(target_throughput) + "kbits/s"
            output += " - throughput: " + str(throughput) + "kbits/s"
            self._logger.info(output)
            # Compare measured and target throughput
            if throughput < target_throughput:
                return Global.FAILURE, output
        else:
            output = str(direction) + " file: " + str(filename) + " successfully"
            output += " - throughput: " + str(throughput) + "kbits/s"
            self._logger.info(output)

        return Global.SUCCESS, output

    @need('wifi or modem or ethernet')
    def start_ftp_xfer(self, direction, server_ip_address,
                       username, password, filename, local_path, loop=None, client_ip_address=""):
        """
        Start a transfer of a file via FTP.

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
        :type local_path: str
        :param local_path: path where to read/save the FTP file (sdcard)
        :type loop: str
        :param loop: True to loop the transfer , False to not used
        :type client_ip_address: str
        :param client_ip_address: IP address of the DUT networking interface we want to use for FTP

        :rtype: operation status & output log & task id
        """
        if local_path is None or local_path == "":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "local FTP directory path is not defined")
        local_dir = os.path.dirname(local_path)
        if self._phone_system.check_directory_exist_from_shell(local_dir) is False:
            self._logger.info("Local path is not valid, create it")
            self._phone_system.create_directory_from_shell(local_dir)
        self._logger.info("Starting FTP transfer ...")

        # create file to upload if not already present in the device
        # Preliminary analysis : create file to upload if not
        # already present in the device (IF POSSIBLE)
        if str(direction).upper() == "UL":
            # analyse file to upload
            try:
                size_kb = self.__file_uecmds.retrieve_size_from_filename(filename)
                self.__file_uecmds.create_file_if_needed(filename, size_kb, local_path)
            except AcsBaseException as excp:
                self._logger.info(
                    "Preliminary analysis failed : we won't be able to "
                    "create file if not already present in the device")
                self._logger.debug("Root cause : " + str(excp))

        # Extract the ftp path and ftp file
        filename = os.path.normpath(filename).replace("\\", "/")
        local_file = os.path.join(local_path, os.path.basename(filename)).replace("\\", "/")

        # Start ftp transfert
        function = "Start_FTP_xfer"
        args = "direction=%s server_ip_address=%s username=%s password=%s remote_filename=%s local_filename=%s client_ip_address=%s" % \
               (direction, server_ip_address, username, password, filename, local_file, client_ip_address)
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, function, args)

        output = str(direction) + " file: " + str(filename) + \
            " successfully started"
        self._logger.info(output)

        return

    def is_ftp_xfer_success(self, filename, direction, task_id):
        """
        Returns FTP transfer success or failure

        :type filename: str
        :param filename: File name to be transfered
        :type direction: str
        :param direction: Transfer direction (UL/DL)
        :type task_id: str
        :param task_id: Transfer to be finished

        :rtype: boolean
        :return: xfer successfull
                - True (success)
                - False (failure or not finished)
        """
        # Start ftp transfert
        function = "Is_FTP_xfer_success"
        args = ""
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        try:
            # Launch the UEcmd on the embedded side
            self._internal_uecmd_exec(module_name, class_name, function, args)
        except DeviceException:
            # if Uecmd returns False (ie FTP transfer is not finished) a DeviceException is raised
            # In this case return just False
            return False
        else:
            return True

    def stop_ftp_xfer(self, task_id, cancel_request=False):
        """
        Stop a transfer of a file via FTP.

        :type task_id: str
        :param task_id: Transfer to be stopped
        :note: this parameter is not used on Windows

        :type cancel_request: bool
        :param cancel_request: Indicate if the stop result form a cancel request
                                (ie timeout) or normal behavior

        :return: None
        """
        self._logger.info("Stopping FTP transfer ...")

        function = "Stop_FTP_xfer"

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        try:
            # Launch the UEcmd on the embedded side
            self._internal_uecmd_exec(module_name, class_name, function)
        except AcsBaseException as ex:
            self._logger.warning(str(ex))

        self._logger.info("FTP stopped")

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
        fct_name = "Connect"
        args = "check_timeout=20 ssid=%s" % (ssid)
        if poor_connection_check:
            self._logger.warning("Poor connection is not checked for windows platforms")
        try:
            module_name, class_name = self._get_module_and_class_names("WifiConnectivity")
            output = self._internal_uecmd_exec(
                module_name,
                class_name,
                fct_name,
                args)
        except AcsBaseException as e:
            if check_connection:
                raise e

    @need('wifi')
    def wifi_disconnect(self, ssid):
        """
        Disconnects from a WIFI network using its SSID

        :type ssid: str
        :param ssid: WIFI network SSID

        :return: None
        """
        self.wifi_disconnect_all()

    @need('wifi')
    def wifi_disconnect_all(self):
        """
        Disconnects from all WIFI networks

        :return: None
        """
        fct_name = "Disconnect"

        module_name, class_name = self._get_module_and_class_names("WifiConnectivity")
        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name)

    # @need('wifi')
    def check_connection_state(self, ssid, timeout=0):
        """
        Checks that the connected SSID is the one passed as parameter

        :type ssid: str
        :param ssid: SSID name of the Wifi network the DUT should be connected
        :type timeout: int
        :param timeout: Customize the timeout
                        (when timeout = 0 --> default value)
        """
        self._logger.info('Networking.check_connection_state not available "as is".'
                          ' Already done when connecting to a wifi network.')
        pass

    # @need('wifi')
    def wifi_remove_config(self, ssid):
        """
        Remove a wifi configuration for the device.

        :type ssid: str
        :param ssid: the ssid of the wifi configuration to be removed or "all"
        """
        self._logger.warning("[NOT IMPLEMENTED] Networking.wifi_remove_config")
        # TODO : remove stub code and implements it

    # @need('wifi')
    def wait_for_wifi_dhcp_connected(self, timeout=None):
        """
        Polls until the IP address has been acquired.
        Then, DHCP state will be "CONNECTED".
        At the end of the default timeout, an exception will be raised

        :type timeout: int
        :param timeout: max number of seconds to wait for IP address
        """
        self._logger.warning("[NOT IMPLEMENTED] Networking.wait_for_wifi_dhcp_connected")

        time.sleep(5)
        # TODO : remove stub code and implements it

    @need('wifi')
    def list_connected_wifi(self):
        """
        Lists the connected WIFI.

        :rtype: list
        :return: The list connected wifi ssid(s)
        """
        fct_name = "ListConnectedWlanNetworks"

        module_name, class_name = self._get_module_and_class_names("WifiConnectivity")
        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name)
        return output["values"]["connected_list"]

    @need('wifi')
    def _get_wifi_security_type(self, security_parameter):
        """
        Get the Windows security type to set on device from the given security.

        :type security_parameter : str
        :param security_parameter: "normalized" security name. Can be NONE, OPEN,
         WEP64-OPEN, WEP128-OPEN, WEP64, WEP128, WPA2-PSK-AES, WPA-PSK-TKIP,
         WPA2-PSK-TKIP, WPA-PSK-AES, WPA-PSK, WPA-PSK-TKIP-AES,
         WPA2-PSK, WPA2-PSK-TKIP-AES, WPA-WPA2-PSK, WPA-WPA2-PSK-TKIP-AES,
         WPA-WPA2-PSK-TKIP, "WPA-WPA2-PSK-AES

        :rtype: str
        :return: Windows specific security name
        """
        # Extract the security Type
        security_upper = str(security_parameter).upper()
        if security_upper in ("NONE", "OPEN"):
            security = "NONE"
        elif security_upper in ("WEP64", "WEP128"):
            security = "WEP"
        elif security_upper in ("WEP64-OPEN", "WEP128-OPEN", "WEP"):
            security = "OPEN"
        elif security_upper in ("WPA2-PSK-AES", "WPA-PSK-TKIP",
                                "WPA2-PSK-TKIP", "WPA-PSK-AES"):
            security = security_upper
        elif security_upper == "WPA-PSK-TKIP-AES":
            # in this case, we must be deterministic
            # (cipher mode is mandatory and can't be set to both TKIP and AES)
            # So we choose the common setting
            security = "WPA-PSK-TKIP"
        elif security_upper == "WPA2-PSK-TKIP-AES":
            # in this case, we must be deterministic
            # (cipher mode is mandatory and can't be set to both TKIP and AES)
            # So we choose the common setting
            security = "WPA2-PSK-AES"
        elif security_upper in ("EAP-WPA", "EAP-WPA2", "EAP-WPA-WPA2",
                                "WPA-PSK", "WPA2-PSK",
                                "WPA-WPA2-PSK", "WPA-WPA2-PSK-TKIP-AES",
                                "WPA-WPA2-PSK-TKIP", "WPA-WPA2-PSK-AES"):
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED,
                                  "Security %s not taken in account yet" % (
                                      security_upper))
        else:
            msg = "Unexpected Wlan security %s!" % str(security_upper)
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

        return security

    @need('wifi')
    def get_wifi_ip_address(self, IPV6=False):
        """
        Returns the Wifi ip address.

        :type IPV6 Boolean
        :param IPV6 False to get IPV4 address, True to get the IPV6 one

        :rtype: str
        :return: wifi ip address
        """
        fct_name = "GetWlanIPAddress"
        args = "is_ipv4=%s" % (str(IPV6 == False).lower())

        module_name, class_name = self._get_module_and_class_names("WifiConnectivity")
        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name,
            args)
        return output["values"]["ip_address"]

    def iperf(self, settings):
        """
        Measures throughputs using IPERF

        :type settings: dictionary
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration...
        Refer to Utilities/IPerfUtilities to get list of possible settings

        :rtype: measured throughput value
        """
        exe = IperfExecutionHandler(settings, self)
        return exe.iperf()

    def start_iperf_client(self, settings):
        """
        Start Iperf client on DUT

        :type settings: dict
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration

        :rtype: measured throughput value
        """
        return self._iperf_client(settings)

    def _iperf_client(self, settings):
        """
        Measures throughputs using iperf as client

        :type settings: dict
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration

        :rtype: measured throughput value
        """
        # Check for mandatory setting
        search_for_mandatory_keyword_arguments(settings, ['server_ip_address', 'port_number', 'duration'])

        port_number = str(settings.get('port_number'))
        duration = str(settings.get('duration'))
        direction = str(settings.get('direction', 'both'))
        protocol = str(settings.get('protocol', 'tcp'))

        msg = "IPERF %s " % protocol.upper()
        if direction == 'both':
            msg += "UL & DL"
        elif direction == 'up':
            msg += "UL"
        else:
            msg += "DL"

        msg += " measurement on port %s for %s seconds" \
               % (str(port_number), str(duration))
        self._logger.info(msg)

        # Set cmd timeout to duration + delta
        # to let the command finishing properly
        # Increase delta_timeout for the multi_slot egprs config
        timeout = (int(duration)) + self._uecmd_default_timeout

        # Build iperf client command
        fct_name = "Iperf"
        args = 'iperf_args="%s" duration=%s' % \
               (IperfOptions(settings).format_iperf_options("client"), timeout)
        module_name, class_name = self._get_module_and_class_names("Networking")

        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name,
            args,
            timeout)

        measures = output["values"]["measures"]
        self._logger.debug("iperf client output:\n" + measures)
        return measures

    def start_iperf_server(self, settings):
        """
        Start Iperf server on DUT

        :type settings: dict
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration
        """
        process = self._start_iperf_server(settings)
        self.__dict__["__process"] = process

    def _iperf_server(self, settings):
        """
        Start Iperf server on DUT and run until the server is closed

        :type settings: dict
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration
        """
        fct_name = "Start_IPerf"
        args = 'iperf_args="%s"' % \
               IperfOptions(settings).format_iperf_options("server")
        module_name, class_name = self._get_module_and_class_names("Networking")
        # Get setting from settings dictionary
        port_number = str(settings.get('port_number'))
        # Run IPERF server through adb
        self._logger.info("Start IPERF server on DUT on port %s:" % port_number)
        self._logger.debug("Send command: " + ' '.join(args))

        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name,
            args)

    def _start_iperf_server(self, settings):
        """
        Launch iperf server on embedded side

        :type settings: dict
        :param settings:  iperf options dictionary coming from user needs, ie:
            { 'iperf':'/data/local/iperf', 'protocol':'tcp'|'udp',
              'direction':'up'|'down'|'both', 'bandwidth' : '11M'|'54M'...,
              'ftp_user': 'ssh_login' (in case 'iperf' is an ssh command) }

        :return: process
        """
        self.kill_iperf()

        self._logger.info("Start IPERF server on DUT:")
        if not isinstance(settings, dict):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Iperf settings not defined correctly.")

        # Check for mandatory setting
        search_for_mandatory_keyword_arguments(settings, ['port_number'])
        # Launch a dedicated process for iperf
        process = iperf_process(self, self._iperf_server, (settings,))
        process.start()
        time.sleep(1)

        return process

    def kill_iperf(self):
        """
        Kill all iperf instance on embedded side
        """
        self._logger.debug("Kill existing iperf server")
        # ensure no iperf server is running
        fct_name = "Kill_IPerf"
        module_name, class_name = self._get_module_and_class_names("Networking")

        self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name)

    def stop_iperf_server(self):
        """
        Stops Iperf server on DUT

        :rtype: str
        :return: Iperf server execution log
        """
        serveroutput = ''
        if hasattr(self, "__process"):
            self._logger.info("Stop IPERF server on DUT:")
            # Terminate IPERF server
            getattr(self, "__process").terminate()

            while True:
                try:
                    line = self._q.get(timeout=.1)
                except Empty:
                    break
                else:
                    serveroutput += str(line)
            self._logger.debug("iperf server output:\n %s" % serveroutput)
        else:
            self._logger.error("Iperf server was not started.")
        return serveroutput

    def stop_iperf(self):
        """
        Send Stop_IPerf Uecmd to DUT and enqueue iperf execution log
        """
        fct_name = "Stop_IPerf"
        module_name, class_name = self._get_module_and_class_names("Networking")
        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            fct_name)
        serveroutput = output["values"]["measures"]
        self._logger.debug("iperf server output:\n %s" % serveroutput)
        self._q.put(output["values"]["measures"])

    def ping(self, server_ip_address, packet_size, packet_count,
             interval=1, flood_mode=False,
             blocking=True, source_address=''):
        """
        Pings from the DUT a server on the bench. the protocol to use (IPv4 or IPv6) is
        computed automatically.

        If you have several interfaces connected (ethernet, wifi, ...), you should want
        to specify on which interface the ping come from. You can do this by specifying
        a source IP address, thanks to source_address input parameter.

        :type server_ip_address: str
        :param server_ip_address: IP address to ping

        :type packet_size: integer
        :param packet_size: Packet size in bytes

        :type packet_count: integer
        :param packet_count: Number of packet to send

        :type interval: float
        :param interval: Interval in seconds between pings (only for IPv4)

        :type flood_mode: Boolean
        :param flood_mode: True if you want to use the ping in flood mode, False otherwise.

        :type blocking: Boolean
        :param blocking: True if you want to throw an error when network is UNREACHABLE.
        The ping will silently fail if this parameter is set to False.

        :type source_address: str
        :param source_address: source IP address to use

        :rtype: Measure object (value,unit)
        :return: packet loss
        """

        self._logger.info("Ping address %s" % server_ip_address)

        # prepare the arguments
        if NetworkingUtil.is_valid_ipv6_address(server_ip_address):
            interval = 1

        args = "server_ip_address=%s packet_size=%s packet_count=%s interval=%s source_address=%s" % (server_ip_address, packet_size, packet_count, interval, source_address)
        function = "ping"
        module_name, class_name = self._get_module_and_class_names("Networking")
        if packet_count > 0:
            timeout = int(packet_count) * (int(interval) + 4)
        else:
            msg = "'packet count' must be an Integer strictly higher than 0"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args, timeout)
        msg_output = output["values"].get("output")
        packetLoss = output["values"].get("packetLoss")
        if not is_number(packetLoss):
            tmp_txt = "ping failed: %s" % str(output)
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        if (-1 == packetLoss):
            error = "Ping command returned an error message (Output = %s)" % output
            if blocking:
                self._logger.error(error)
                raise DeviceException(DeviceException.DEFAULT_ERROR_CODE, error)
            else:
                self._logger.info(error)
                ret = Measure()
                ret.value = float(-1)
                return ret

        ret = Measure()
        ret.value = float(packetLoss)
        ret.units = "%"
        return ret

    def ping6(self, ip_address, packet_size, packet_count, flood_mode=False,
              blocking=True, source_address=None):
        """
        Pings IPV6 addresses from the DUT a server on the bench

        If you have several interfaces connected (ethernet, wifi, ...), you should want
        to specify on which interface the ping come from. You can do this by specifying
        a source IP address, thanks to source_address input parameter.

        :type ip_address: str
        :param ip_address: IP address to ping

        :type packet_size: integer
        :param packet_size: Packet size in bytes

        :type packet_count: integer
        :param packet_count: Number of packet to send

        :type interval: float
        :param interval: Interval in seconds between pings

        :type flood_mode: Boolean
        :param flood_mode: True if you want to use the ping in flood mode, False otherwise.

        :type blocking: Boolean
        :param blocking: True if you want to throw an error when network is UNREACHABLE.
        The ping will silently fail if this parameter is set to False.

        :type source_address: str
        :param source_address: source IP address to use

        :rtype: Measure object
        :return: packet loss
        """
        return self.ping(ip_address, packet_size, packet_count,
                         flood_mode=flood_mode, blocking=blocking,
                         source_address=source_address)

    def set_preferred_network_type(self, preferred_type):
        """
        Sets the Preferred Network Type .

        :type preferred_type:  str
        :param preferred_type: can be:
            "2G_ONLY"       for # 1: "GSM_ONLY",
            "3G_PREF"       for # 0: "WCDMA_PREF",
            "4G_PREF"       for # 9: "LTE_GSM_WCDMA",
            "3G_ONLY"       for # 2: "WCDMA_ONLY",
            "2G_3G"         for # 3: "GSM_UMTS",
            "CDMA_PREF"     for # 4: "CDMA",
            "CDMA_ONLY"     for # 5: "CDMA_NO_EVDO"
            "EVDO_ONLY"     for # 6: "EVDO_NO_CDMA"
            "GLOBAL"        for # 7: "GLOBAL",
            "4G_PREF_US"    for # 8: "LTE_CDMA_EVDO",
            "WORLD_MODE"    for # 10: "LTE_CMDA_EVDO_GSM_WCDMA"
            "4G_ONLY"       for # 11: "LTE_ONLY"
            "WIN_0x12345678" for non-standard network type
        :return: None
        """

        preferred_type = self.get_untranslated_network_type(preferred_type)
        modem_api = self._device.get_uecmd("Modem", True)
        self.last_set_data_class = preferred_type
        modem_api.set_preferred_radio(preferred_type)

    def get_preferred_network_type(self):
        """
        Returns the Preferred Network Type.

        :rtype: str
        :return:
            "2G_ONLY"       for # 1: "GSM_ONLY",
            "3G_PREF"       for # 0: "WCDMA_PREF",
            "4G_PREF"       for # 9: "LTE_GSM_WCDMA",
            "3G_ONLY"       for # 2: "WCDMA_ONLY",
            "2G_3G"         for # 3: "GSM_UMTS",
            "CDMA_PREF"     for # 4: "CDMA",
            "CDMA_ONLY"     for # 5: "CDMA_NO_EVDO"
            "EVDO_ONLY"     for # 6: "EVDO_NO_CDMA"
            "GLOBAL"        for # 7: "GLOBAL",
            "4G_PREF_US"    for # 8: "LTE_CDMA_EVDO",
            "WORLD_MODE"    for # 10: "LTE_CMDA_EVDO_GSM_WCDMA"
            "4G_ONLY"       for # 11: "LTE_ONLY"
            "WIN_0x12345678" for non-standard network type
        """

        modem_api = self._device.get_uecmd("Modem", True)
        # to check gcdc value is inside the the value in set preferred
        time.sleep(15)  # window dut take little longer to get the current data class after register
        gcdc_value = modem_api.get_preferred_radio()
        self._logger.info("checking and convert gcdc_value %d to prefered data class %d" % (gcdc_value, self.last_set_data_class))
        if self.last_set_data_class and gcdc_value:
            gcdc_value = self.last_set_data_class

        gcdc_value = self.get_translated_network_type(gcdc_value)
        return gcdc_value

    @need('wifi')
    def set_autoconnect_mode(self, interface, state):
        """
        Sets the autoconnect mode to on/off for a specific interface or all

        :type interface: str
        :param interface: interface to modify or 'all' for all interfaces

        :type state: str or int
        :param state: Can be ("on", "1" , 1) to enable autoconnect
                             ("0", "off" , 0) to disable autoconnect

        :return: None
        """
        mode = "unknown"
        if state is AUTO_CONNECT_STATE.on or state in ("on", "1", 1):
            mode = "true"
        elif state is AUTO_CONNECT_STATE.off or state in ("0", "off", 0):
            mode = "false"
        else:
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "unknown state: '%s'" % state)

        params = "ssid=%s mode=%s" % (interface, mode)
        method = "SetAutoconnectMode"
        module_name, class_name = self._get_module_and_class_names("WifiConnectivity")
        # no need to retrieve output: uecmd result is automatically handled
        self._internal_uecmd_exec(
            module_name,
            class_name,
            method,
            params)

    @need('wifi')
    def get_interface_mac_addr(self, interface="wifi"):
        """
        Returns the MAC address of the given interface.

        :type interface: str
        :param interface: the interface name.

        :rtype: str
        :return: The dut mac address
        """
        method = "GetMACAddress"
        module_name, class_name = self._get_module_and_class_names("WifiConnectivity")
        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            method)

        wifi_mac_address = str(output["values"]["mac_address"])
        if not NetworkingUtil.is_valid_mac_address(wifi_mac_address):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Failed to get wifi MAC address until %d seconds" %
                                  self._uecmd_default_timeout)
        return wifi_mac_address

    # @need('wifi')
    def get_wifi_frequency_band(self, silent_mode=False, _interface="wlan0"):
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
        :return: The band text (JB or later).

        """
        self._logger.warning("[NOT IMPLEMENTED] Networking.get_wifi_frequency_band")
        # todo : implements it ! (and remove stubbed code)
        return self.__mock_freq_band

    def disable_output_traffic(self):
        """
        Prevent all traffic including icmp

        :param: None
        :return: None
        """
        self._logger.info("Disable all output traffic")

        network_interface = self._device.get_cellular_network_interface()
        local_ip_address = self.get_interface_ipv4_address(network_interface)
        # set rule to block traffic and apply soon
        cmd = "netsh advfirewall firewall add rule name=acs-traffic-conrol dir=out action=block enable=yes localip=%s " % local_ip_address
        output = self._internal_exec(cmd)
        self._logger.info("Disable all output traffic done!")

    def enable_output_traffic(self):
        """
        Allow UE traffic to go out

        :param: None
        :return: None
        """
        self._logger.info("Enable all output traffic")
        # set remove the rule
        cmd = "netsh advfirewall firewall delete rule name=acs-traffic-conrol"
        output = self._internal_exec(cmd)
