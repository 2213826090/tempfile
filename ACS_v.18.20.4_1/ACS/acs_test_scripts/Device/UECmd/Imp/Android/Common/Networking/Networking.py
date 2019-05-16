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
:since: 21/03/2011
:author: sfusilie, jreynaud
"""

import time
import re
import os
import random
from datetime import datetime
import tempfile
import posixpath
from Queue import Empty

from UtilitiesFWK.Utilities import Global, run_local_command, str_to_bool_ex
from acs_test_scripts.Utilities.IPerfUtilities import \
    IperfOptions, IperfExecutionHandler, search_for_mandatory_keyword_arguments

from acs_test_scripts.Device.UECmd.UECmdTypes import Measure, AUTO_CONNECT_STATE, WidiDevice, NETWORK_PREFERENCES, \
    PreferredNetwork
from acs_test_scripts.Device.UECmd.Interface.Networking.INetworking import INetworking
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiKeyExchangeTypes
from string import digits  # pylint: disable=W0402
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil

from acs_test_scripts.Utilities.NetworkingUtilities import SupplicantState
from acs_test_scripts.Utilities.RegistrationUtilities import CellInfo, get_dict_key_from_value
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.AcsToolException import AcsToolException
from acs_test_scripts.Utilities.IPerfUtilities import Iperf


class Networking(BaseV2, INetworking, Iperf):
    """
    Class that handle all networking operations
    """
    # Constant Android values for WIFI_SLEEP_POLICY
    WIFI_SLEEP_POLICY = {"DEFAULT": 0x0, "WHEN_SCREEN_OFF": 0x0,
                         "NEVER": 0x2, "NEVER_WHILE_PLUGGED": 0x1}

    # Constant for FTP transfer status
    FTP_TRANSFERRING = "transferring"

    # Constants that refer to the existing Wifi chipsets
    CHIPSET_UNKNOWN = "Chipset Unknown"
    CHIPSET_BROADCOM = "Chipset_Broadcom"
    CHIPSET_TI = "Chipset_TI"
    CHIPSET_INTEL = "Chipset_INTEL"

    DEFAULT_REG_DOMAIN = "FR"
    """
    Conversion table between generic preferred network type and android preferred network type
    """
    NETWORK_TYPE_CONVERSION_TABLE = \
        {
            0: "3G_PREF",  # 0: "WCDMA_PREF",
            1: "2G_ONLY",  # 1: "GSM_ONLY",
            2: "3G_ONLY",  # 2: "WCDMA_ONLY",
            3: "2G_3G",  # 3: "GSM_UMTS",
            4: "CDMA_PREF",  # 4: "CDMA",
            5: "CDMA_ONLY",  # 5: "CDMA_NO_EVDO"
            6: "EVDO_ONLY",  # 6: "EVDO_NO_CDMA"
            7: "GLOBAL",  # 7: "GLOBAL",
            8: "4G_PREF_US",  # 8: "LTE_CDMA_EVDO",
            9: "4G_PREF",  # 9: "LTE_GSM_WCDMA",
            10: "WORLD_MODE",  # 10: "LTE_CMDA_EVDO_GSM_WCDMA"
            11: "4G_ONLY",  # 11: "LTE_ONLY"
        }

    def is_preferred_network_type_valid(self, network_type):
        """
        Checks whether the network type is valid.

        :type network_type: str
        :param network_type: can be one of NETWORK_PREFERENCES list elements

        :rtype: bool
        :return: True for valid or False for invalid network type
        """
        res = False
        if network_type in NETWORK_PREFERENCES:
            res = True
        return res

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        INetworking.__init__(self, device)
        Iperf.__init__(self)

        self._logger = device.get_logger()
        self._phone_system = device.get_uecmd("PhoneSystem")
        self.__file_uecmds = device.get_uecmd("File")
        self.icategory = "intel.intents.category.NETWORKING"
        self.component = "com.intel.acs.agent/.Networking"
        self.UECMD_ERROR_NETWORK_UNREACHABLE = "Network is unreachable"

        self.ftp_module = "acscmd.transfer.FtpModule"
        self.network_type_module = "acscmd.telephony.TelephonyModule"
        self._wifi_module = "acscmd.connectivity.wifi.WifiModule"
        self._tethering_module = "acscmd.connectivity.TetheringModule"
        self._connectivity_module = "acscmd.connectivity.ConnectivityModule"
        self.__web_browser_module = "acscmd.web.WebBrowserModuleActivity"
        self._cellular_networking_module = "acscmd.telephony.CellularNetworkingModule"
        self._continuous_ping_module = "acscmd.transfer.ContinuousPingModule"
        self._http_transfer_module = "acscmd.transfer.HttpTransferModule"

        self._wifi_chipset_manufacturer = None
        self.__last_set_reg_domain = "none"

    @need('wifi', False)
    def set_wifi_power(self, mode):
        """
        Sets the WIFI power to off or on.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        # We first release any previously acquired lock
        self._logger.debug("set_wifi_power_saving_mode to 'on' "
                           "in order to release previously acquired lock.")
        self._set_wifi_power_saving_mode("1")

        # We check the current WIFI power status
        current_mode = str(self.get_wifi_power_status())

        # We go on with the actual execution
        if str_to_bool_ex(mode):
            # We simply update the local variable
            # because the power saving mode has already been set
            mode = "1"
            self._logger.info("Setting wifi power to on")
            self._set_wifi_power_saving_mode("0")
            warning_msg = "Wifi Power is already on"

        elif str_to_bool_ex(mode) is False:
            # We update the local variable and change
            # the power saving mode state
            mode = "0"
            self._logger.info("Setting wifi power to off")
            self._set_wifi_power_saving_mode("1")
            warning_msg = "Wifi Power is already off"

        else:
            msg = "set_wifi_power : Parameter mode %s is not valid" % mode
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # We call the UE Command
        if current_mode == mode:
            self._logger.info(warning_msg)

        else:
            method = "setWifiPower"
            cmd_args = "--ei mode %s" % mode
            self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

            # Restore previous regulatory domain on DUT as it is not persistent
            if (mode == "1") and (self.__last_set_reg_domain != "none"):
                self._logger.info("Restoring previous regulatory domain (%s)"
                                  % self.__last_set_reg_domain)
                self.set_regulatorydomain(self.__last_set_reg_domain)

    @need('wifi', False, 0)
    def get_wifi_power_status(self):
        """
        Gets the WIFI power.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        method = "getWifiPowerStatus"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)
        self._logger.info("Wifi power status: " + output["wifi_power_status"])

        return int(output["wifi_power_status"])

    @need('wifi')
    def set_wificonfiguration(self, ssid, pass_phrase, security,
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
        self._logger.info(
                "Setting passphrase %s for ssid %s (security level : %s)"
                % (str(pass_phrase), str(ssid), str(security)))

        # Prepare variable for control
        security_upper = self._get_wifi_security_type(str(security).upper())

        if security_upper in ("NONE", "OPEN"):
            # Set variable to be used for Embd
            security = "None"
        elif security_upper in ("WEP", "WEP-OPEN"):
            # Set variable to be used for Embd
            security = security_upper
        elif security_upper.startswith("WPA"):
            # Set variable to be used for Embd for both WPA and WPA2
            security = "WPA"
        elif security_upper in ("EAP-WPA", "EAP-WPA2"):
            # Set variable to be used for Embd
            security = security_upper
        else:
            # If security does not contain valid information
            output = "set_wificonfiguration : Wrong parameter security: %s" % str(security)
            self._logger.error(output)
            raise AcsConfigException(AcsConfigException.DEFAULT_ERROR_CODE, output)

        self._logger.debug("%s Security Wifi SSID" % security)

        method = "setWifiConfiguration"
        cmd_args = "--es ssid %s --es pass_phrase %s --es security %s " % (ssid, pass_phrase, security)

        # Add args for static ip configurations
        if ip_method != '':
            cmd_args += '--es ip_method %s ' % ip_method
            cmd_args += '--es static_ip "%s" ' % address
            cmd_args += '--es netmask "%s" ' % netmask
            cmd_args += '--es gateway "%s" ' % gateway
            cmd_args += '--es dns1 "%s" --es dns2 "%s" ' % (dns1, dns2)

        # Add args for proxy configurations
        cmd_args += '--es proxy_config %s ' % proxy_config
        if proxy_config != "NONE":
            cmd_args += '--es proxy_address %s ' % proxy_address
            cmd_args += '--es proxy_port %s ' % proxy_port
            cmd_args += '--es proxy_bypass %s ' % proxy_bypass

        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

    @need('wifi')
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
        # pylint: disable=E1101

        self._logger.info("Setting autoconnect mode for %s interfaces to %s" % (str(interface), str(state)))

        # Check that we are asking for a known state
        if state not in AUTO_CONNECT_STATE:
            # Unknown request state
            output = "set_autoconnect_mode : %s is not in known state" % state
            self._logger.error(output)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, output)
        elif state == AUTO_CONNECT_STATE.off:
            mode = "0"
        else:
            mode = "1"

        # Get cellular interface
        cellular_ssid = self._phone_system.get_single_sim_gsm_property_value("gsm.sim.operator.alpha")

        # Set autoconnect mode on cellular interface
        if interface in (cellular_ssid, "all"):
            if mode == "0":
                self.deactivate_pdp_context()
            else:
                self.activate_pdp_context()

        # Set autoconnect mode on wifi interface
        method = "setAutoconnectMode"
        cmd_args = "--es ssid %s --ei mode %s" % (interface, mode)
        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

    @need('wifi', False)
    @need('modem', False)
    def clean_all_data_connections(self):
        """
        Disable PDP context and remove all known Wifi networks

        :return: None
        """
        self._logger.info("Clean all data connections")

        self.deactivate_pdp_context()
        self.wifi_remove_config("all")

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

        method = "wifiConnect"
        cmd_args = "--es ssid %s --ei poorConnectionCheck %d" % (ssid, poor_connection_check)

        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)
        if check_connection:
            self.check_connection_state(ssid)

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

        connection_state = False
        timeout = int(timeout)
        if timeout <= 0:
            timeout = self._uecmd_default_timeout
        start = time.time()
 
        while (connection_state == False
               and not start + timeout < time.time()):
            time.sleep(15)

            # List connected SSIDs to check if the right ssid is connected
            connected_wifi_list = self.list_connected_wifi()

            if ssid in connected_wifi_list:
                self._logger.info("Wifi connected to network %s" % str(ssid))
                connection_state = True
                break

        if not connection_state:
            raise DeviceException(DeviceException.TIMEOUT_REACHED, "Wifi connect to %s timeout !" % ssid)

    @need('wifi')
    def wifi_disconnect(self, ssid):
        """
        Disconnects from a WIFI network using its SSID

        :param ssid: WIFI network SSID

        :return: None
        """
        self._logger.info("Disconnecting from wifi network %s" % str(ssid))
        method = "wifiDisconnect"
        cmd_args = "--es ssid %s" % ssid
        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

    @need('wifi')
    def wifi_disconnect_all(self):
        """
        Disconnects from all WIFI network

        :return: None
        """
        self._logger.info("Disconnecting from all wifi networks")

        method = "wifiDisconnectAll"
        self._internal_exec_v2(self._wifi_module, method, is_system=True)

    @need('modem')
    def set_apn(self, interface=None, apn=None, user=None, password=None, protocol=None, mmsc=None, apn_type=None,
                set_default=True, clear_apn=True):
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
        if apn is None:
            apn = "default"

        if interface is None:
            name = "'" + self._phone_system. \
                get_single_sim_gsm_property_value("gsm.operator.alpha") + "'"
            if name in ("", None):
                name = "default"
        else:
            name = interface

        # Remove existing APNs if needed
        if clear_apn:
            self.clear_apn()

        self._logger.info("Create a new APN '" + str(apn) +
                          "' on " + str(interface) + " interface")

        # Get the MCC & the MNC from the USIM
        sim_operator = self._phone_system.get_single_sim_gsm_property_value("gsm.sim.operator.numeric")

        sim_mcc = sim_operator[:3]
        sim_mnc = sim_operator[3:]

        check_null = ("", None)
        if (sim_mcc in check_null) or (sim_mnc in check_null):
            output = "sim_mcc or sim_mnc not found !"
            self._logger.error(output)
            return

        target_method = "setApnParameters"
        cmd_args = "--es name %s --es mcc %s --es mnc %s" % (interface, str(sim_mcc), str(sim_mnc))
        # Add to command line only input parameters which are not None
        # to keep Android UI parameters as not set in APN menu
        input_args = ['apn', 'user', 'password', 'protocol', 'mmsc', 'apn_type', 'set_default']
        for param in input_args:
            var = eval(param)
            if var is not None:
                if type(var) is int:
                    cmd_args += " --ei %s %d" % (param, int(eval(param)))
                elif type(var) is bool:
                    cmd_args += " --ez %s %s" % (param, str(eval(param)).lower())
                elif type(var) is str:
                    cmd_args += " --es %s %s" % (param, str(eval(param)))

        if protocol is None:
            self._logger.debug("No APN protocol will be set")
        elif protocol in ("IP", "IPV6", "IPV4V6"):
            cmd_args += " --es protocol " + str(protocol) \
                        + " --es roaming_protocol " + str(protocol)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The protocol should be either IP, IPV6 or IPV4V6 not %s" % protocol)
        self._internal_exec_v2(self._cellular_networking_module, target_method, cmd_args, is_system=True)

    @need('modem')
    def get_apn(self):
        """
        Get the current APN stored on DUT
        :rtype: String
        :return: Name of the APN currently set on the phone.
        """
        self._logger.info("Getting current APN parameters")
        target_method = "getApnParameters"
        output = self._internal_exec_multiple_v2(self._cellular_networking_module, target_method, is_system=True)

        apn = None
        # Go through the data stored in the carriers table, in order to get the name of the APN
        # set on the phone
        for element in output:
            if "apn" in element:
                apn = element["apn"]

        return apn

    @need('modem')
    def clear_apn(self):
        """
        Clear the current APN configuration on the DUT
        :return: None
        """
        self._logger.info("Removing current APN parameters... ")
        target_method = "deleteApnParameters"
        output = self._internal_exec_multiple_v2(self._cellular_networking_module, target_method, is_system=True)

        # Let the user know which APN(s) were removed
        if "FAILURE" in output.pop()['status']:
            self._logger.info("No APN previously configured, nothing to do here")

        for element in output:
            if "name" in element.keys():
                self._logger.info("APN %s deleted successfully" % element['name'])

    @need('modem', False)
    def activate_pdp_context(self, interface=None, check=True):
        """
        Activates a Packet Data Protocol (PDP) context using its interface

        .. note:: interface parameter not used on Android
        :type interface: str
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context activation is checked (no check for GSM)

        :return: None
        """
        self._logger.info("Checking PDP Context Status")
        pdp_context_status = self._get_pdp_context_status()
        if pdp_context_status in ("0", "1"):
            if pdp_context_status == "0":
                self._logger.info("PDP Context Enabled but not yet connected")
            elif pdp_context_status == "1":
                self._logger.info("PDP Context Enabled and already connected")
                return

        elif pdp_context_status == "2":
            self._logger.info("PDP Context not enabled, enabling ...")
            # try to wake up screen durin 10 seconds
            self._phone_system.wake_screen()
            i = 0
            while (not self._phone_system.get_screen_status()) and i < 10:
                time.sleep(1)
                i += 1
                self._phone_system.wake_screen()
            if i == 10:
                msg = "Unable to Wake UP screen in 10s"
                self._logger.error(msg)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)
            # Unlock the device to keep the PDP alive
            self._phone_system.set_phone_lock(0)
            method = "setPdpContextStatus"
            arg = "--ei state 1"
            self._internal_exec_v2(self._cellular_networking_module, method, arg, is_system=True)

        # Check activation
        if check:
            time_count = 0
            connection_state = False
            while ((connection_state == False) and
                           time_count <= self._uecmd_default_timeout):

                pdp_context_status = self._get_pdp_context_status()

                if pdp_context_status == "1":
                    connection_state = True
                    break

                time.sleep(1)
                time_count += 1

            if not connection_state:
                msg = "Active PDP context timeout !"
                self._logger.error(msg)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)

    @need('modem', False)
    def reactivate_pdp_context(self, apn_name, check=True):
        """
        Check if PDP is activated, if not activate it (no need to do it on android)

        :note: apn_name parameter not used on Windows
        :type interface: str
        :param interface: Data Cellular interface

        :type check: boolean
        :param check: if PDP context activation is checked (no check for GSM)

        :return: None
        """
        pass

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
        self._logger.info("Checking PDP Context Status")

        pdp_context_status = self._get_pdp_context_status()

        if pdp_context_status == "2":
            self._logger.info("PDP Context already disabled")
            return

        elif pdp_context_status in ("0", "1"):
            if pdp_context_status == "0":
                self._logger.info("PDP Context enabled but not connected")
            if pdp_context_status == "1":
                self._logger.info("PDP Context enabled and connected")

            self._logger.info("Disabling PDP Context ...")
            method = "setPdpContextStatus"
            arg = "--ei state 0"
            self._internal_exec_v2(self._cellular_networking_module, method, arg, is_system=True)

        # Checking deactivation
        if check:
            time_count = 0
            connection_state = False
            while ((connection_state == False) and
                           time_count <= self._uecmd_default_timeout):

                pdp_context_status = self._get_pdp_context_status()

                if pdp_context_status == "2":
                    connection_state = True
                    break
                time.sleep(1)
                time_count += 1

            if not connection_state:
                error_msg = "Deactivate pdp context timeout !"
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, error_msg)

    @need('modem')
    def _get_pdp_context_status(self):
        """
        Returns the PDP Context status on the DUT

        :rtype: String
        :return: the pdp context status
        """
        method = "getPdpContextStatus"
        output = self._internal_exec_v2(self._cellular_networking_module, method, is_system=True)

        return str(output["pdp_context_status"])

    def set_flight_mode(self, mode):
        """
        Sets the flight mode to off or on.
        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        :return: None
        """
        mode = str_to_bool_ex(mode)
        if mode:
            mode = "1"
            self._logger.info("Enabling flight mode ...")
            warning_msg = "Flight mode is already enabled"
        elif mode == False:
            mode = "0"
            self._logger.info("Disabling flight mode ...")
            warning_msg = "Flight mode is already disabled"
        else:
            error_msg = "set_flight_mode : Parameter mode %s is not valid" % mode
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        current_mode = str(self.get_flight_mode())

        if current_mode == mode:
            self._logger.info(warning_msg)
        else:
            module = "acscmd.connectivity.ConnectivityModule"
            method = "setFlightMode"
            cmd_args = "--ei mode %s" % mode
            self._internal_exec_v2(module, method, cmd_args, is_system=True)

    def get_flight_mode(self):
        """
        Returns the flight mode.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        module = "acscmd.connectivity.ConnectivityModule"
        method = "getFlightMode"
        output = self._internal_exec_v2(module, method, is_system=True)

        return int(output["flight_mode_state"])

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
        :param filename: File name to be transferred
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

        # Check if 'local_path' parameter points to a valid location
        if self._phone_system.check_directory_exist_from_shell(local_path) == False:
            self._logger.info("Local path is not valid, create it")
            self._phone_system.create_directory_from_shell(local_path)

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

        self._logger.info("Starting FTP transfer ...")
        # Start ftp transfer
        method = "startFtpXfer"

        cmd_args = "--es direction %s --es url %s" \
                   " --es username %s --es password %s --es localPath '%s'" \
                   % (str(direction), str(server_ip_address),
                      str(username), str(password), local_path.replace("\\", "/"))

        # Extract the ftp path and ftp file
        filename = os.path.normpath(filename).replace("\\", "/")
        ftp_file = os.path.basename(filename)
        ftp_path = os.path.dirname(filename)

        # Add ftp file to command to send
        cmd_args += " --es file %s" % str(ftp_file)
        # Check if the file will transferred into a specific directory
        if ftp_path != "":
            cmd_args += " --es ftpPath %s" % str(ftp_path)

        if loop is not None and isinstance(loop, bool):
            if loop:
                loop = 1
            else:
                loop = 0
            cmd_args += " --es loop %s" % str(loop)

        self._internal_exec_with_retry_v2(self.ftp_module, method, cmd_args, is_system=True)

        output = str(direction) + " file: " + str(filename) + \
                 " successfully started"
        self._logger.info(output)

        # Retrieve the task_id of the start_ftp_xfer method
        task_id = self._last_tag_op_code
        self._device_logger.add_trigger_message(task_id)

        return task_id

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
                - False (failure)
        """
        # Consider only the file name without any potential path
        filename = os.path.basename(filename)

        success_msg = "Ftp %s of file %s finish success !" % (direction, filename)
        triggered_status = str(self._device_logger.get_message_triggered_status(task_id))
        self._device_logger.remove_trigger_message(task_id)

        # When the  success message  is in the message of ACS op code
        if success_msg in str(triggered_status):
            return True
        else:
            return False

    def stop_ftp_xfer(self, task_id, cancel_request=False):
        """
        Stop a transfer of a file via FTP.

        :type task_id: str
        :param task_id: Transfer to be stopped
        .. note:: this parameter is not used on Android

        :type cancel_request: bool
        :param cancel_request: Indicate if the stop result form a cancel request
                                (ie timeout) or normal behavior

        .. warning:: On Android the implemenation consist to broadcast the intent:
                    acs.intents.action.STOP_FTP

        :return: None
        """
        self._logger.info("Stopping FTP transfer ...")

        self._device_logger.remove_trigger_message(str(task_id))

        method = "stopFtpXfer"

        cmd_args = ""
        if cancel_request:
            cmd_args = "--ei cancelRequest 1"

        try:
            self._internal_exec_v2(self.ftp_module, method, cmd_args, is_system=True)
        except AcsBaseException as ex:
            self._logger.warning(str(ex))

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

        method = "getFtpXferStatus"

        output = self._internal_exec_v2(self.ftp_module, method, is_system=True)

        return output["output"].strip()

    def ftp_xfer(self, direction, server_ip_address,
                 username, password, filename, timeout, local_path, target_throughput=None, client_ip_address="",
                 multi_session=False):
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

        :type multi_session: bool
        :param multi_session: allows multiple simultaneous ftp transfer (one UL and one DL)

        :rtype: list
        :return: operation status & output log
        """
        if local_path is None or local_path == "":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "local FTP directory path is not defined")

        # Check if 'local_path' parameter points to a valid location
        if not self._phone_system.check_directory_exist_from_shell(local_path):
            self._logger.info("Local path is not valid, create it")
            self._phone_system.create_directory_from_shell(local_path)

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
        if username is "" and password is "":
            username = "anonymous"
            password = ""
        if multi_session is None:
            multi_session = False
        # Start ftp transfer
        method = "ftp"
        cmd_args = "--es direction %s --es url %s" \
                   " --es username %s --es password '%s' --es localPath '%s' --ez multidir '%s'" \
                   % (str(direction), str(server_ip_address),
                      str(username), str(password), local_path.replace("\\", "/"), str(multi_session).lower())

        # Extract the ftp path and ftp file
        filename = os.path.normpath(filename).replace("\\", "/")
        ftp_file = os.path.basename(filename)
        ftp_path = os.path.dirname(filename)

        # Add ftp file to command to send
        cmd_args += " --es file %s" % str(ftp_file)
        # Check if the file will transferred into a specific directory
        if ftp_path != "":
            cmd_args += " --es ftpPath %s" % str(ftp_path)

        try:
            ftp_result = self._internal_exec_with_retry_v2(self.ftp_module, method, cmd_args,
                                                           timeout=timeout, is_system=True)
        except AcsBaseException as error:
            # Return FTP transfer error when timeout on uecommand result is reached
            if error.get_error_message().find(self.UECMD_TIMEOUT_RESULT_MSG) != -1:
                self._logger.info("FTP Transfer timeout, stopping remaining transfer")
                error_msg = "FTP Transfer timeout !"

                try:
                    self.stop_ftp_xfer(None, True)
                except AcsBaseException as stop_error:
                    # Log warning message in case stop ftp fails
                    warning_msg = "Stop FTP transfer failed ! (%s)" % stop_error.get_error_message()
                    self._logger.warning(warning_msg)

                # Raise FTP transfer timeout
                raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error_msg)

            else:
                # Raise the uecommand exception with the root cause
                raise error

        throughput = ftp_result.get("throughput", "NA")

        # If a target throughput has been given, check if measured throughput is greater or equal than the target value
        if target_throughput is not None:
            # Measured througput is returned in kBytes/sec
            tput_list = throughput.split("kBytes/sec")
            tput = tput_list[0]
            output = str(direction) + " file: " + str(filename) + " successfully"
            output += " - target throughput: " + str(target_throughput) + "kbits/s"
            # Convert measured throughput to kbits/s
            if tput.isdigit():
                tput_val = float(tput) * 8
                output += " - throughput: " + str(tput_val) + "kbits/s"
            else:
                output += "!!! WRONG UNIT in measured throughput, it shall be kBytes/sec !!!! - throughput: " + str(
                    throughput)
                return Global.FAILURE, output
            self._logger.info(output)
            # Compare measured and target throughput
            if tput_val < target_throughput:
                return Global.FAILURE, output
        else:
            output = str(direction) + " file: " + str(filename) + " successfully"
            output += " - throughput: " + str(throughput)
            self._logger.info(output)

        return Global.SUCCESS, output

    def kill_iperf(self):
        """
        Kill all iperf instance on embedded side
        """
        self._logger.debug("Kill existing iperf server")
        # ensure no iperf server is running
        cmd = "adb shell killall iperf"
        self._exec(cmd, 10)
        time.sleep(1)

    def start_iperf_server(self, settings):
        process, q = self._start_iperf_server(settings)
        self.__dict__["__process"] = process
        self.__dict__["__q"] = q

    def _start_iperf_server(self, settings):
        """
        Launch iperf server on embedded side
        .. warning:: Using a compiled version (2.0.2) of iperf by intel on.
                    !!!! The executable should be present in /data/local

        :type settings: dict
        :param settings:  iperf options dictionary coming from user needs, ie:
            { 'iperf':'/data/local/iperf', 'protocol':'tcp'|'udp',
              'direction':'up'|'down'|'both', 'bandwidth' : '11M'|'54M'...,
              'ftp_user': 'ssh_login' (in case 'iperf' is an ssh command) }

        :return: tuple (process, q)
        """

        self.kill_iperf()

        if not isinstance(settings, dict):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Iperf settings not defined correctly.")

        # Check for mandatory setting
        search_for_mandatory_keyword_arguments(settings, ['port_number'])

        # Get setting from settings dictionary
        port_number = str(settings.get('port_number'))

        # Build the iperf command
        iperf_destination = self._device.get_device_os_path().join(self._device.binaries_path, 'iperf')
        if len(self._device.get_serial_number()) != 0:
            largs = ['adb', '-s', self._device.get_serial_number(), 'shell', iperf_destination]
        else:
            largs = ['adb', 'shell', iperf_destination]
        largs += IperfOptions(settings).format_iperf_options("server").split(' ')

        # Run IPERF server through adb
        self._logger.info("Start IPERF server on DUT on port %s:" % port_number)
        self._logger.debug("Send command: " + ' '.join(largs))
        (process, q) = run_local_command(largs)
        time.sleep(1)

        return process, q

    def stop_iperf_server(self):
        serveroutput = ''
        if hasattr(self, "__process") and hasattr(self, "__q"):
            self._logger.info("Stop IPERF server on DUT:")
            # Terminate IPERF server
            getattr(self, "__process").terminate()

            while True:
                try:
                    line = getattr(self, "__q").get(timeout=.1)
                except Empty:
                    break
                else:
                    serveroutput += str(line)
            self._logger.debug("iperf server output:\n %s" % serveroutput)
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
        lost_packet = "out-of-order"
        serveroutput = ""
        self._logger.debug("Looking for: " + lost_packet + " in the iperf command queue.")
        while True:
            try:
                line = queue.get(timeout=.1)
            except Empty:
                break
            else:
                serveroutput += line
        if lost_packet in serveroutput:
            self._logger.info("Lost packet detected.")
            return True
        else:
            self._logger.info("No lost packet detected.")
            return False

    def start_iperf_client(self, settings):
        return self._iperf_client(settings)

    def _iperf_client(self, settings):
        """
        Measures throughputs using iperf as client

        .. warning:: Using a compiled version (2.0.2) of iperf by intel on.
                    !!!! The executable should be present in /system/bin

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

        # Build iperf client command
        iperf_destination = self._device.get_device_os_path().join(self._device.binaries_path, 'iperf')
        cmd = "adb shell " + iperf_destination + " "
        cmd += IperfOptions(settings).format_iperf_options("client")
        self._logger.info("IPERF CMD=" + cmd)

        # Set cmd timeout to duration + delta
        # to let the command finishing properly
        # Increase delta_timeout for the multi_slot egprs config
        timeout = (int(duration)) + self._uecmd_default_timeout

        output = self._exec_with_retry(cmd, timeout, uecmd_error_type=self.UECMD_ERROR_NETWORK_UNREACHABLE)

        self._logger.debug("iperf client output:\n" + output)
        return output

    def iperf(self, settings):
        """
        Measures throughputs using IPERF

        .. warning:: Using a compiled version (2.0.2) of iperf by intel on.
                    !!!! The executable should be present in /system/bin

        :type settings: dictionary
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration

        :rtype: measured throughput value
        """
        exe = IperfExecutionHandler(settings, self)
        return exe.iperf()

    def iperf_async(self, settings):
        """
        Start Iperf Async

        type settings: dictionary
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration
        """
        exe = IperfExecutionHandler(settings, self)
        exe.iperf_async()
        return exe

    def ping(self,
             server_ip_address,
             packet_size,
             packet_count,
             interval=1,
             flood_mode=False,
             blocking=True,
             source_address=None):
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
        self._logger.info("Ping address " + str(server_ip_address) +
                          " with " + str(packet_count) + " packets of " +
                          str(packet_size) + " bytes...")
        if packet_count > 0:
            timeout = int(packet_count) * 10
        else:
            self._logger.error("'packet count' must be an Integer > 0")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "'packet count' must be an Integer > 0")

        # Compute whether ip_address is V4 or V6 format.
        if NetworkingUtil.is_valid_ipv6_address(str(server_ip_address)):
            self._logger.warning("Interval parameter is ignored for IPv6")
            # Ping in v6 format
            cmd = "adb shell ping6 " + \
                  " -s " + str(packet_size) + \
                  " -c " + str(packet_count) + \
                  " " + str(server_ip_address)
        elif (NetworkingUtil.is_valid_ipv4_address(str(server_ip_address)) or
                  (str(server_ip_address) != "")):
            # Either Ping in V4 format
            # Or if we fall here, this is because the ip is a hostname,
            # so we do a classic ping
            cmd = "adb shell ping" + \
                  " -s " + str(packet_size) + \
                  " -c " + str(packet_count) + \
                  " -i " + str(interval) + \
                  " " + str(server_ip_address)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Empty IP address provided")

        output = self._exec_with_retry(cmd, timeout,
                                       uecmd_error_type=self.UECMD_ERROR_NETWORK_UNREACHABLE)
        # Print output results
        self._logger.info(output)

        output_lower = output.lower()
        packet_loss = Measure()

        if "network is unreachable" not in output_lower:
            output_match = re.search("received,[+,0-9A-Za-z ]* ([0-9]+\.?[0-9]*)% packet loss", output_lower)

            if output_match is None:
                error = "Ping results are missing (Output = %s)" % output
                self._logger.error(error)
                raise DeviceException(DeviceException.DEFAULT_ERROR_CODE, error)

            # Parse output to get measured % of packets lost
            self._logger.debug("Parsed packet loss rate: " + str(output_match.groups()))
            packet_loss.value = float(output_match.group(1))
            packet_loss.units = "%"

        else:
            error = "Ping command returned an error message (Output = %s)" \
                    % output
            if blocking:
                raise DeviceException(DeviceException.DEFAULT_ERROR_CODE, error)
            else:
                self._logger.info(error)
                packet_loss.value = -1
                return packet_loss

        return packet_loss

    def ping6(self,
              ip_address,
              packet_size,
              packet_count,
              flood_mode=False,
              blocking=True,
              source_address=None):
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

        self._logger.info("Ping address " + str(ip_address) + " with " + str(packet_count) +
                          " packets of " + str(packet_size) + " bytes...")
        if packet_count > 0:
            timeout = int(packet_count) * 10
        else:
            self._logger.error("'packet count' must be an Integer > 0")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "'packet count' must be an Integer > 0")

        cmd = "adb shell busybox ping6 " + \
              " -s " + str(packet_size) + \
              " -c " + str(packet_count) + \
              " " + str(ip_address)

        output = self._exec_with_retry(cmd, timeout,
                                       uecmd_error_type=self.UECMD_ERROR_NETWORK_UNREACHABLE)

        output_lower = output.lower()
        output_split = output.split(",")
        packet_loss = Measure()

        if output_lower.find("network is unreachable") == -1:
            if len(output_split) < 3:
                error = "Ping results are missing (Output = %s)" % output
                self._logger.error(error)
                raise DeviceException(DeviceException.DEFAULT_ERROR_CODE, error)

            if output_lower.find("error") == -1:
                output_string = output_split[2].strip()
            else:
                output_string = output_split[3].strip()

            # Parse output to get measured % of packets lost
            packet_loss.value = float(output_string.split('%')[0])
            packet_loss.units = "%"

        else:

            error = "Ping command returned an error message (Output = %s)" \
                    % output
            if blocking:
                self._logger.error(error)
                raise DeviceException(DeviceException.DEFAULT_ERROR_CODE, error)

            else:
                self._logger.info(error)
                packet_loss.value = -1
                return packet_loss

        return packet_loss

    def start_continuous_ping(self, ip_address):
        """
        Start continuous ping a to given address

        :type ip_address: str
        :param ip_address: IP address to ping

        :return: None
        """
        method = "startPing"
        cmd_args = "--es ipAddress %s" % ip_address
        self._internal_exec_v2(self._continuous_ping_module, method, cmd_args, is_system=True)

    def stop_continuous_ping(self):
        """
        Stop the continuous ping ongoing

        :return: None
        """
        method = "stopPing"
        self._internal_exec_v2(self._continuous_ping_module, method, is_system=True)

    def get_current_continuous_ping(self):
        """
        Get the current ping result of the ongoing continuous ping

        :rtype: (bool, float)
        :return: (ping success, ping rtt)
        """
        method = "getCurrentPing"
        output = self._internal_exec_v2(self._continuous_ping_module, method, is_system=True)
        ping_success = str_to_bool_ex(output["success"])
        rtt = float(output["rtt"])
        self._logger.debug("Ping : " + str((ping_success, rtt)))
        return ping_success, rtt

    def check_continuous_ping_success(self, duration):
        """
        Checks that ongoing continuous ping is successful

        :type duration: float
        :param duration: duration of the check in seconds

        :rtype: (bool, float)
        :return: (result of the check, average rtt)
        """
        result = True
        nb_ping = 0
        sum_rtt = 0.
        start_time = time.clock()
        while time.clock() - start_time < duration:
            (ping_success, rtt) = self.get_current_continuous_ping()
            if ping_success:
                nb_ping += 1
                sum_rtt += rtt
            else:
                result = False
        if result:
            average_rtt = sum_rtt / float(nb_ping)
        else:
            average_rtt = 0
        return result, average_rtt

    def check_continuous_ping_failure(self, duration):
        """
        Checks that ongoing continuous ping is failing

        :type duration: float
        :param duration: duration of the check in seconds

        :rtype: bool
        :return: result of the check
        """
        result = True
        start_time = time.clock()
        while time.clock() - start_time < duration:
            (ping_success, _) = self.get_current_continuous_ping()
            if ping_success:
                result = False
        return result

    @need('modem or wifi or nfc or bluetooth or gps', False)
    def get_available_technologies(self):
        """
        Returns an array of available technologies.

        :rtype: list
        :return: List of technologies to activate if available
                                - cellular
                                - wifi
                                - bluetooth
                                - gps
                                - fm
        """
        interfaces = []

        self._logger.info("Getting available wireless technologies ...")

        # Order in which technologies are checked is very important.
        # It drives in which order those technos are ON/OFF for Power measurement UCs

        self._phone_system.display_on()

        try:
            # Check modem
            if self._device.is_capable("modem"):
                modem_status = self._device.get_property_value("init.svc.ril-daemon")
                if modem_status in (None, ""):
                    self._logger.warning("init.svc.ril-daemon key not found or empty")
                else:
                    if modem_status in ("running", "stopped"):
                        interfaces.append("cellular")
            else:
                self._logger.debug("Modem is not set in device capabilities")

            # Check wifi
            if self._device.is_capable("wifi"):
                wifi_status = self._device.get_property_value("wifi.interface")
                if wifi_status in (None, ""):
                    self._logger.warning("wifi.interface key not found or empty")
                else:
                    interfaces.append("wifi")
            else:
                self._logger.debug("Wifi is not set in device capabilities")

            # Check bluetooth
            if self._device.is_capable("bluetooth"):
                bt_status = self._device.get_property_value("net.bt.name")
                if bt_status in (None, ""):
                    self._logger.warning("net.bt.name key not found or empty")
                else:
                    interfaces.append("bluetooth")
            else:
                self._logger.debug("Bluetooth is not set in device capabilities")

            # Check gps, determinist according DeviceModel
            if self._device.is_capable("gps"):
                gps_key = self._device.get_gps_property_key()
                gps_status = self._device.get_property_value(gps_key)
                if gps_status in (None, ""):
                    self._logger.warning(gps_key + " key not found or empty")
                else:
                    interfaces.append("gps")
            else:
                self._logger.debug("GPS is not set in device capabilities")

        finally:
            self._phone_system.display_off()

        return interfaces

    @need('wifi')
    def list_connected_wifi(self):
        """
        Lists the connected WIFI.

        :rtype: list
        :return: the requested value
        """
        wifi_list = self.list_ssids("wifi", "connected")

        return wifi_list

    @need('wifi')
    def get_wifi_power_level(self):
        """
        Returns the Wifi TX power level in dBm.

        :rtype: float
        :return: wifi power in dBm
        """
        method = "getWifiPowerLevel"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)
        rvalue = output["wifi_power_level"]

        return float(rvalue)

    @need('wifi')
    def set_wifi_power_saving_mode(self, mode, interface=None):
        """
        Sets the power saving mode of the given wlan interface to the given value.
        This method calls the internal C{_set_wifi_power_saving_mode} method
        and adds some additional checks and log messages.

        .. warning:: 'interface' parameter isn't used on Android.
        .. warning:: we can't ensure that set_wifi_power_mode to
        ON is correctly functionnal because it depends on other
        android applications using wifi radio.

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

        .. warning:: 'interface' parameter isn't used on Android.
        .. warning:: we can't ensure that set_wifi_power_mode to
        ON is correctly functionnal because it depends on other
        android applications using wifi radio.

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
        method = "setWifiPowerSavingMode"
        cmd_args = "--ei mode %s" % mode
        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

    @need('wifi')
    def get_interface_mac_addr(self, interface="wifi"):
        """
        Returns the MAC address of the given interface.
        .. warning:: unused parameter 'interface'.

        :type interface: str
        :param interface: the interface name.

        :rtype: str
        :return: The dut mac address
        """
        method = "getWifiMACAddress"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)

        # Search for result
        wifi_mac_address = output["mac_address"]

        if not NetworkingUtil.is_valid_mac_address(wifi_mac_address):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Failed to get wifi MAC address until %d seconds" % self._uecmd_default_timeout)

        return str(wifi_mac_address)

    @need('wifi')
    def request_wifi_scan(self):
        """
        Trigger a Wifi scan.

        :return: None
        """
        method = "requestWifiScan"
        self._internal_exec_v2(self._wifi_module, method, is_system=True)

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
                         - "remembered"
                         - "all" for any state

        :rtype: list
        :return: a list of ssids
        """
        return self._list_ssids(technology, state, False, False)

    @need('wifi')
    def list_ssids_and_capabilities(self, technology="wifi", state="all"):
        """
        Lists the ssids and capabilities for a given technology and a given state.
        .. warning:: technology parameter unused (only wifi ssids available).

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :type state: str
        :param state: the state of the ssid to list value can be
                         - "connected"
                         - "disconnected"
                         - "all" for any state

        :rtype: list
        :return: a list of ssids and capabilities
        """
        return self._list_ssids(technology, state, True, False)

    @need('wifi')
    def list_ssids_and_frequency(self, technology="wifi", state="all"):
        """
        Lists the ssids and frequency for a given technology and a given state.
        .. warning:: technology parameter unused (only wifi ssids available).

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :type state: str
        :param state: the state of the ssid to list value can be
                         - "connected"
                         - "disconnected"
                         - "all" for any state

        :rtype: list
        :return: a list of ssids and frequency
        """
        return self._list_ssids(technology, state, False, True)

    @need('wifi')
    def _list_ssids(self, technology="wifi", state="all", return_capabilities=False, return_frequency=False):
        """
        Lists the ssids and :
        - capabilities (if return_capabilities = true)
        - frequency (if return_frequency = true)
        for a given technology and a given state.
        .. warning:: technology parameter unused (only wifi ssids available).

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :type state: str
        :param state: the state of the ssid to list value can be
                         - "connected"
                         - "disconnected"
                         - "remembered"
                         - "visible"
                         - "all" for any state (visible and remembered)

        :type return_capabilities: str
        :param return_capabilities: True to return the Capabilities.

        :type return_frequency: str
        :param return_frequency: True to return the Frequencies.

        :rtype: tuple
        :return: a tuple of lists ssids and :
        - capabilities (if return_capabilities = true) - Describes the authentication,
        key management, and encryption schemes supported by the access point
        - frequency (if return_frequency = true) - The frequency in MHz of the
        channel over which the client is communicating with the access point
        """
        if technology not in ("wifi", "all"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "%s technology not supported !" % technology)

        if state not in ("connected", "disconnected", "remembered", "visible", "all"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "%s state not supported !" % state)

        method = "listSSIDs"
        cmd_args = "--es mode %s" % state

        raw_ssids = self._internal_exec_multiple_v2(self._wifi_module, method, cmd_args, is_system=True)

        ssids = self._build_list_from_dict(raw_ssids, "ssid")

        if return_capabilities and return_frequency:
            capabilities = self._build_list_from_dict(raw_ssids, "capabilities")
            frequency = self._build_list_from_dict(raw_ssids, "frequency")
            return ssids, capabilities, frequency

        if return_capabilities:
            capabilities = self._build_list_from_dict(raw_ssids, "capabilities")
            return ssids, capabilities

        if return_frequency:
            frequency = self._build_list_from_dict(raw_ssids, "frequency")
            return ssids, frequency

        return ssids

    @need('wifi')
    def check_ssid_bfore_timeout(self, ssid, timeout, technology="wifi"):
        """
        Try to find a given ssid before timeout.

        .. warning:: Wifi is the only one technology available

        :type ssid: str
        :param ssid: ssid to check

        :type timeout: int
        :param timeout: operation timeout

        :type technology: str
        :param technology: the technology type like "wifi"... or "all" to target all kind of technologies

        :rtype: boolean
        :return: research result
        """
        msg = "ssid %s not found after!" % ssid
        ssid_found = False
        scanning_time = 0

        while scanning_time <= timeout:
            self.request_wifi_scan()
            # wait 2 seconds instead of 1,
            # in order to ensure that scan
            # is done (scan interval on android >= 2s)
            time.sleep(2)
            scanning_time += 2

            ssids = self.list_ssids(technology, "all")

            if ssid in ssids:
                ssid_found = True
                msg = "ssid %s found!" % ssid
                break

        self._logger.info(msg)
        return ssid_found

    @need('wifi')
    def get_wifi_dhcp_state(self):
        """
        Returns the Wifi DHCP state according to android NetworkInfo.State.

        :rtype: str
        :return: wifi DHCP state
        """
        dhcp_status = "ERROR"
        method = "getWifiDHCPState"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)

        if 'wifi_dhcp_state' not in output:
            raise AcsToolException(AcsToolException.PHONE_OUTPUT_ERROR, "get wifi DHCP state failed")
        else:
            dhcp_status = output["wifi_dhcp_state"]

        return str(dhcp_status)

    @need('wifi')
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

    @need('wifi')
    def get_wifi_ip_address(self, IPV6=False):
        """
        Returns the Wifi ip address.

        :type IPV6 Boolean
        :param IPV6 False to get IPV4 address, True to get the IPV6 one

        :rtype: str
        :return: wifi ip address
        """
        if not IPV6:
            method = "getWifiIPAddress"
        else:
            method = "getWifiIPV6Address"

        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)

        ip_address = None
        # Search for the 1st element containing an ip address (we don't support multiple ipv4 addresses).
        for cur_elem in output.keys():
            if 'wifi_ip_address' in cur_elem:
                ip_address = output[cur_elem]
                break
        if ip_address is None:
            raise AcsToolException(AcsToolException.PHONE_OUTPUT_ERROR, "get wifi ip address failed")

        if '/' in ip_address:
            re.sub(r'.*/([^%]+).*', r'\1', ip_address)
        # Simplify empty groups
        ip_address = re.sub('(:0{1,4})+:', '::', ip_address)
        return str(ip_address)

    @need('wifi or ethernet')
    def get_interface_ipv4_address(self, interface):
        """
        Returns the ipv4 address of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: str
        :return: interface ip address
        """
        cmd = "adb shell busybox ifconfig " + interface
        output = self._exec(cmd)
        m = re.search(r"inet addr:([0-9.]+) ", output)
        if m is None:
            msg = "Could not get IPV4 address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        interface_ip = m.groups()[0]
        return interface_ip

    @need('wifi or ethernet')
    def get_interface_ipv4_all_address(self):
        """
        Returns ipv4 addresses for all active network interface.

        :rtype: dict
        :return: {active interface name: ip address}
        """
        cmd = "adb shell netcfg "
        output = self._exec(cmd)
        """
        netcfg returns a list of all interface (active or not) with the following format:
        if_name  state(active or nor)                 IP address
        lo       UP                                   127.0.0.1/8   0x00000049 00:00:00:00:00:00
        sit0     DOWN                                   0.0.0.0/0   0x00000080 00:00:00:00:00:00
        usb0     DOWN                                   0.0.0.0/0   0x00001082 00:00:00:00:00:00
        usb1     DOWN                                   0.0.0.0/0   0x00001002 00:00:00:00:00:00
        usb2     DOWN                                   0.0.0.0/0   0x00001002 00:00:00:00:00:00
        usb3     DOWN                                   0.0.0.0/0   0x00001002 00:00:00:00:00:00
        usb4     DOWN                                   0.0.0.0/0   0x00001002 00:00:00:00:00:00
        wlan0    DOWN                                   0.0.0.0/0   0x00001002 00:13:20:ff:35:f7
        p2p-wlan0-0 UP                            192.168.49.12/0   0x00001002 00:14:66:ff:36:f8
        we generate a dictionary with all active (UP) interface name: ip address
        """
        if_dict = dict(re.findall(r"([a-zA-Z0-9-]*) +UP +([0-9.]+)", output))
        if if_dict == {}:
            msg = "No active interface with IPV4 address"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        return if_dict

    @need('wifi')
    def set_wifi_hotspot(self, state, hotspot_ssid="", hotspot_security="", hotspot_passphrase="", hotspot_standard="",
                         hotspot_channel="", hotspot_hidden="false"):
        """
        Set the SSID, security and password for the Configurable wifi hotspot.

        :type state: str
        :param state: wifi hotspot enable or disable: on|off
        :type hotspot_ssid: str
        :param hotspot_ssid: ssid of hotspot (required only if state=on)
        :type hotspot_security: str
        :param hotspot_security: security of hotspot, can be
                         OPEN|WPA-PSK|WPA2-PSK (required only if state=on)
        :type hotspot_passphrase: str
        :param hotspot_passphrase: password of hotspot,
                         (required only if state=on and hotspot_security!=OPEN)
        :type hotspot_standard: str
        :param hotspot_standard: standard of hotspot, optional, can be (2_4GHZ_20MHZ;5GHZ_20MHZ;5GHZ_40MHZ;5GHZ_80MHZ)
        :type hotspot_channel: str
        :param hotspot_channel: channel of hotspot, value "AUTO" by default (only if standard is used, optional)
        :type hotspot_hidden: str
        :param hotspot_hidden: hidden SSID
        """
        self._logger.info("Trying to set wifi hotspot")
        state = str(state).lower()
        if state not in ("on", "off"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "'state' Parameter must be 'on' or 'off'.")
        if state == "on" and (hotspot_ssid == "" or hotspot_security == ""):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "'hotspot_ssid' and 'hotspot_security' Parameters must be defined")
        if state == "on" and hotspot_security != "OPEN" and hotspot_passphrase == "":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "'hotspot_passphrase' Parameter must be defined")

        method = "setWifiHotspot"
        cmd_args = "--es ssid '%s' --es security '%s'  --es passphrase '%s' --es switch '%s'" \
                   % (hotspot_ssid, hotspot_security, hotspot_passphrase, state)
        self._internal_exec_v2(self._tethering_module, method, cmd_args, is_system=True)

    @need('wifi')
    def get_wifi_hotspot_status(self):
        """
        Get the status of the Wifi hotspot feature.

        :rtype: int
        :return: 1 if enabled, 0 if disabled
        """
        method = "getWifiHotspotStatus"
        output = self._internal_exec_v2(self._tethering_module, method, is_system=True)
        self._logger.info("Wifi tethering status: " + output["wifi_hotspot_status"])

        return int(output["wifi_hotspot_status"])

    @need('wifi')
    def wifi_remove_config(self, ssid):
        """
        Remove a wifi configuration for the device.

        :type ssid: str
        :param ssid: the ssid of the wifi configuration to be removed or "all"
        """
        # retrieve the initial wifi power status
        wifi_initial_status = self.get_wifi_power_status()

        # Enable wifi if necessary, to remove a known wifi network
        if wifi_initial_status == 0:
            self._logger.info("In order to remove remembered wifi "
                              + "networks, we must power on the wifi")
            self.set_wifi_power("on")

        # Remove the requested SSID
        self._logger.info("Trying to remove wifi config [%s]" % ssid)
        method = "removeWifiConfig"
        cmd_args = "--es ssid %s" % ssid
        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

        # reset wifi state to its original state
        if wifi_initial_status == 0:
            self._logger.info("Set the wifi power to its original state")
            self.set_wifi_power("off")

    def open_web_browser(self, website_url, browser_type="native", timeout=None, skip_eula=False):
        """
        Open the Android Browser on the web page
        passed as parameter.

        :type website_url: str
        :param website_url: URL to open

        :type browser_type: str
        :param browser_type: "native" will open the default browser,
                             "acs_agent" will use the browser of the acs agent

        :type timeout: int
        :param timeout: timeout to open the page

        :type skip_eula: boolean
        :param skip_eula: skip EULA on 1st start

        :rtype: tuple
        :return: operation status & output log
        """

        self._logger.info("Opening the %s web browser and load url %s"
                          % (str(browser_type), str(website_url)))

        error_code = Global.SUCCESS
        error_msg = "No errors"

        # Check timeout parameter
        if timeout is None:
            timeout = self._uecmd_default_timeout

        if not isinstance(timeout, int) or timeout <= 0:
            error_code = Global.FAILURE
            error_msg = "Parameter timeout must be integer superior to 0"
            self._logger.error("open_web_browser : Bad timeout value :%s", timeout)

            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        if browser_type == "native":
            if skip_eula:
                self._logger.debug("No EULA on native browser")
            cmd = "adb shell am start"
            action = "android.intent.action.VIEW"
            category = "android.intent.category.BROWSABLE"
            component = "com.android.browser/.BrowserActivity"
            cmd += " -a %s -c %s -n %s -d \"%s\"" \
                   % (action, category, component, str(website_url))

            self._exec(cmd, timeout)
            error_code = Global.SUCCESS

            error_msg = "Intent to open the native web browser has been sent." \
                        " Check manually that the web page has been loaded correctly"
            self._logger.info(error_msg)

        elif browser_type == "chrome":
            if skip_eula:
                self._skip_chrome_eula()

            cmd = "adb shell am start"
            action = "android.intent.action.VIEW"
            component = "com.android.chrome/com.google.android.apps.chrome.Main"
            cmd += " -a %s -t text/html -n %s -d \"%s\"" \
                   % (action, component, str(website_url))
            output = self._exec(cmd, timeout).splitlines()
            time.sleep(5)
            if len(output) > 1:
                error_code = Global.FAILURE
                error_msg = "Failed to launch browser: %s" % output[1]

        elif browser_type == "acs_agent":
            if skip_eula:
                self._logger.debug("No EULA on acs agent browser")
            method = "loadUrl"
            args = '--es url "%s"' % website_url
            try:
                self._internal_exec_v2(self.__web_browser_module, method, args, is_system=False)
            except AcsBaseException as e:
                error_code = Global.FAILURE
                error_msg = str(e)

        else:
            error_code = Global.FAILURE
            error_msg = "Unsupported browser type : %s" % str(browser_type)
            self._logger.error("open_web_browser : %s", error_msg)

        return error_code, error_msg

    def _skip_chrome_eula(self):
        """
        Skip chrome eula
        """
        config_dir = "/data/data/com.android.chrome/shared_prefs"
        config_file = "/data/data/com.android.chrome/shared_prefs/" \
                      "com.android.chrome_preferences.xml"
        config = "<?xml version='1.0' encoding='utf-8' standalone='yes' ?>\n" \
                 "<map>\n" \
                 "<boolean name='first_run_eula_accepted' value='true' />\n" \
                 "<boolean name='first_run_flow' value='true' />\n" \
                 "</map>"
        status, _ = self._device.run_cmd("adb shell mkdir -p {0}; echo \"{1}\" > {2}".format(config_dir,
                                                                                             config,
                                                                                             config_file),
                                         10)
        if status != Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Cannot skip chrome EULA!")

    def close_web_browser(self, browser_type="native"):
        """
        Close the Android Browser.

        :type browser_type: str
        :param browser_type: "native" will open the default browser,
            other type can be added depending on the os

        :return: None
        """
        self._logger.info("Closing the %s web browser" % str(browser_type))

        if browser_type == "native":
            # Exit the com.android.browser/.BrowserActivity application
            self._exec("adb shell am force-stop com.android.browser")
        elif browser_type == "chrome":
            self._exec("adb shell am force-stop com.android.chrome")
        elif browser_type == "acs_agent":
            # Build command to close web browser
            method = "closeWebBrowser"
            self._internal_exec_v2(self.__web_browser_module, method, is_system=False)
        else:
            error_msg = "Unsupported browser type : %s" % str(browser_type)
            self._logger.error("open_web_browser : %s", error_msg)
            raise AcsConfigException(
                    AcsConfigException.INVALID_PARAMETER, error_msg)

    @need('widi')
    def widi_scan_devices(self):
        """
        Scans remote WiDi adapter.

        :rtype: list of WidiDevice object
        :return: list of founded adapters (addresses only)
        """
        self._logger.info("Scan for WiDi devices ...")
        # retrieve WIDI scan result from logcat
        # Because we can't directly access to the result
        # in the ACS embedded part, we watch the logcat
        # until an implementation in ACS_Agent is possible

        return self.__retrieve_widi_scan_from_logcat()

    @need('widi')
    def __retrieve_widi_scan_from_logcat(self):
        """
        Retrieve widi scan from logcat

        :rtype: list
        :return: list of C{WidiDevice} containing only mac addresses
        """
        status, src_device_list = self.listen_to_logcat(
                ["widi-p2p", "wpa_supplicant"],
                "API SEARCH_COMPLETE signal",
                None,
                self._uecmd_default_timeout)

        # notify that scan is incomplete if listen_to_logcat status is false
        if not status:
            self._logger.warning("Retrieve incomplete log scan from logcat")

        # extract MAC addresses from retrieved messages
        expr_mac = ".*(?P<addr>([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2})"
        expr_full = expr_mac + ".*name \'(?P<name>([-a-zA-Z0-9]*))-(?P<adapter>([a-zA-Z0-9]*))\' config.*"

        devices_list = []

        for device in src_device_list:
            matches_str = re.compile(expr_full).search(device)
            if matches_str is not None:
                # address found
                address = matches_str.group("addr")
                name = matches_str.group("name")
                adapter = matches_str.group("adapter")

                self._logger.debug("Found: " + str(name) + " (" + address + ") in " + adapter)

                wd = WidiDevice()
                wd.address = address
                wd.name = name
                wd.adapter = adapter

                devices_list.append(wd)

        # Remove duplicate
        self._logger.debug("Remove duplicate in WiDi devices list ...")
        if devices_list:
            devices_list.sort()
            last = devices_list[-1]
            for i in range(len(devices_list) - 2, -1, -1):
                if last.name == devices_list[i].name and \
                                last.address == devices_list[i].address and \
                                last.adapter == devices_list[i].adapter:
                    del devices_list[i]
                else:
                    last = devices_list[i]

        return devices_list

    @need('wifi', False)
    def push_wpa_certificate(self, certificate_file):
        """
        Push given certificate file from host to device.

        :param certificate_file: str
        :param certificate_file: Certificate file to push onto target
        """

        if os.path.exists(certificate_file):
            # Remove existing certificates from device
            self._exec("adb shell rm {0}/*.p12".format(self._device.get_sdcard_path()))

            # Push the certificate into the sdcard file system
            dest_cert_file = posixpath.join(self._device.get_sdcard_path(), os.path.basename(certificate_file))
            self._exec('adb push "{0}" "{1}"'.format(certificate_file, dest_cert_file))
        else:
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "Certificate file {0} not found on host !".format(certificate_file))

    @need('wifi', False)
    def load_wpa_certificate(self, certificate_name=None,
                             certificate_file=None,
                             eap_password=None,
                             credential_password=None):
        """
        Load the WPA certificate file from the SDCARD
        Prerequisite: A certificate file ".p12" should have been pushed into
        the folder /sdcard/.
        Warning, only 1 .p12 file must be present on the SDCARD
        Warning 2: if a credential password is set, it should be the PIN code
                    specified in the benchConfig (Credential_password)

        :type certificate_name: str
        :param certificate_name: Name to give to the certificate after loading

        :type certificate_file: str
        :param certificate_file: Name of the certificate file to load

        :type eap_password: str
        :param eap_password: password to open the certificate file

        :type credential_password: str
        :param credential_password: password to set for the Android credential
                                    passwords

        :return: None
        """
        KEYCODE_DPAD_UP = "19"
        KEYCODE_DPAD_DOWN = "20"
        KEYCODE_DPAD_CENTER = "23"
        KEYCODE_HOME = "3"
        display = self._device.get_uecmd("Display")

        # Unlock the device
        self._phone_system.set_phone_lock(0)

        # Force the screen orientation to portrait
        display.set_display_orientation("portrait")

        # Open Location and Security Settings directly
        self._exec(
                "adb shell am start -n com.android.settings/.SecuritySettings")

        # Erase all credential items and password
        count = 0
        while count < 15:
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
            count += 1
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Run the certificate installation
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Enter the certificate password
        self._exec("adb shell input text " + eap_password)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Validate the name of the certificate
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Set a credential password
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input text " + credential_password)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input text " + credential_password)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Go back to home screen
        self._exec("adb shell input keyevent " + KEYCODE_HOME)

        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        display.set_display_orientation("auto")

    @need('wifi')
    def set_wifi_sleep_policy(self, policy):
        """
        Set the wifi sleep policy

        :type policy: WIFI_SLEEP_POLICY enum
        :param policy: policy to set
        .. seealso:: http://developer.android.com/reference/android/provider/Settings.System.html#WIFI_SLEEP_POLICY
        """
        if not isinstance(policy, int) or policy not in self.WIFI_SLEEP_POLICY.values():
            error_msg = "set_wifi_sleep_policy(): Wrong parameter: " + str(policy)
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Send the intent to set the Wifi sleep policy
        method = "setWifiSleepPolicy"
        cmd_args = "--ei policy %s" % str(policy)
        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

    @need('wifi')
    def get_wifi_sleep_policy(self):
        """
        get the wifi sleep policy

        :return: the wifi sleep policy
        .. seealso:: http://developer.android.com/reference/android/provider/Settings.System.html#WIFI_SLEEP_POLICY
        """
        method = "getWifiSleepPolicy"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)
        policy = int(output["wifi_sleep_policy"])
        if policy not in self.WIFI_SLEEP_POLICY.values():
            self._logger.warning("get_wifi_sleep_policy(): read value from " +
                                 "Android SDK: %s. " % policy +
                                 "Known values in ACS Host:" +
                                 " %s" % str(self.WIFI_SLEEP_POLICY.values()))

        return policy

    @need('wifi')
    def wifi_menu_settings(self, displayed=True):
        """
        Enter WiFi menu setting or exit from WiFi menu settings
        This UEcmd is useful for WiFi policy tests

        :type displayed: bool
        :param displayed: True to enter into WiFi menu setting\
                            and False to exit (goes to idle screen)
        """
        # Go to idle menu
        self._exec("adb shell am start -c android.intent.category.HOME -a android.intent.action.MAIN")

        if displayed:
            # Open WiFi setting menu
            self._exec("adb shell am start " +
                       "-n com.android.settings/.wifi.WifiSettings")

    @need('wifi')
    def wifi_menu_advanced_settings(self):
        """
        Enter WiFi menu advanced setting or exit from WiFi menu advanced settings
        """
        self._exec("adb shell am start " + "-a android.settings.WIFI_IP_SETTINGS")

    def redirect_log_on_dut(self):
        """
        Start logging logcat into a file on DUT
        """

        # ensure no redirected logcat is already running
        self.kill_log_on_dut()

        file_name = "/data/log_dhcp_renew_associated.txt"
        script_name = "/data/redirect_logcat.sh"
        script_data = \
            """#!/system/bin/sh\\n
logcat -c\\n
logcat -v time > %s &\\n
echo \$! > /data/logcat_pid.txt""" % file_name

        cmd = "adb shell echo \"%s\" > %s" % (script_data, script_name)
        self._exec(cmd)

        # Set execution permissions
        cmd = "adb shell chmod 777 %s" % script_name
        self._exec(cmd)

        # Run script detached on DUT
        cmd = "adb shell exec nohup %s > /dev/null && echo -n" % script_name
        self._exec(cmd, wait_for_response=False)

    def kill_log_on_dut(self):
        """
        Stop logging logcat into a file on DUT
        """
        cmd = "adb shell kill `cat /data/logcat_pid.txt`"
        self._exec(cmd)

        cmd = "adb shell rm /data/redirect_logcat.sh /data/logcat_pid.txt"
        self._exec(cmd)

    @need('wifi or modem or ethernet')
    def retrieve_dhcp_renewal_interval_on_dut(self, nb_hits):
        """
        Computes the average dhcp renewal interval for nb_hits renewals

        :type nb_hits: int
        :param nb_hits: number of renewals for wich the average has to be computed

        :rtype: int or None
        :return: average renewal interval for nb_hits renewals or None if operation failed
        """
        if not isinstance(nb_hits, int) or nb_hits <= 0:
            msg = "retrieve_dhcp_renewal_interval(): Wrong nb_hits parameter: " + str(nb_hits)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        file_name = "/data/log_dhcp_renew_associated.txt"
        cmd = "adb shell cat %s" % file_name
        output = self._exec(cmd)

        patterns = r'^(.*dhcp.*renewing lease.*)$|^(.*dhcp.*leased.*)$'
        result = re.findall(patterns, output, re.MULTILINE)

        # Sanity check: accept one more lease
        if len(result) / 2 < nb_hits or len(result) / 2 > nb_hits + 1:
            msg = "Wrong dhcp renewal count found in logcat: %d instead of %d" % (len(result) / 2, nb_hits)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Extract leases
        renewals = []
        i = 0
        while i < nb_hits * 2:
            start = result[i][0]
            i += 1
            if len(start) != 0:
                stop = result[i][1]
                i += 1
                renewals.append(stop)

        self._logger.info("Renewals: %d", len(renewals))

        # compute time between each lease
        intervals = []
        for i in range(nb_hits - 1):
            # extract the time in each renew
            interval = datetime.strptime(renewals[i + 1][6:14], '%H:%M:%S') - \
                       datetime.strptime(renewals[i][6:14], '%H:%M:%S')
            intervals.append(interval.seconds)

        # compute the average for intervals and return it
        time_duration = 0
        for interval in intervals:
            time_duration += interval
        average = time_duration / (nb_hits - 1)

        self._logger.info("Average: %d", int(average))

        return average

    @need('wifi or modem or ethernet')
    def retrieve_dhcp_renewal_interval(self, nb_hits, timeout):
        """
        Computes the average dhcp renewal interval for nb_hits renewals

        :type nb_hits: int
        :param nb_hits: number of renewals for wich the average has to be computed

        :type timeout: int
        :param timeout: max number of seconds between 2 renewals

        :rtype: int or None
        :return: average renewal interval for nb_hits renewals or None if operation failed
        """
        key = "executing `/system/etc/dhcpcd/dhcpcd-run-hooks', reason RENEW"
        renewals = []
        intervals = []

        error_msg = None
        if not isinstance(nb_hits, int) or nb_hits <= 0:
            error_msg = "retrieve_dhcp_renewal_interval(): Wrong nb_hits parameter: " \
                        + str(nb_hits)

        if not isinstance(timeout, int) or timeout <= 0:
            error_msg = "retrieve_dhcp_renewal_interval(): Wrong timeout parameter: " \
                        + str(timeout)

        if error_msg is not None:
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        for i in range(nb_hits):
            status, lease_renewal = self.listen_to_logcat("/system/bin/dhcpcd",
                                                          key, None, timeout)
            # notify that scan is incomplete if listen_to_logcat status is false
            if not status:
                self._logger.warning("Retrieve dhcp renewal from logcat failed")
                return None
            else:
                renewals.append(lease_renewal)

        # compute time between each lease
        for i in range(nb_hits - 1):
            renew1 = ""
            renew2 = ""
            # extract correct time
            for renewal in renewals[i]:
                if key in renewal:
                    # we consider this line
                    renew1 = renewal
            for renewal in renewals[i + 1]:
                if key in renewal:
                    # we consider this line
                    renew2 = renewal
            # extract the time in each renew
            interval = datetime.strptime(renew2[6:14], '%H:%M:%S') - \
                       datetime.strptime(renew1[6:14], '%H:%M:%S')
            intervals.append(interval.seconds)

        # compute the average for intervals and return it
        time_duration = 0
        for interval in intervals:
            time_duration += interval
        average = time_duration / (nb_hits - 1)
        return average

    @need('wifi')
    def set_regulatorydomain(self, regulatory_domain, _interface="wlan0"):
        """
        Set the Wifi Regulatory Domain

        :type regulatory_domain: String
        :param regulatory_domain: the regulatory domain to set (FR, GB, US...)
        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        if regulatory_domain.lower() == "none":
            regulatory_domain = "00"

        # Force the root because sometimes it is lost and the set
        # regulatory domain command fails.
        self._exec("adb root", force_execution=True, timeout=10)
        time.sleep(2)

        wifi_chipset = self._get_wifi_chipset_manufacturer()

        if wifi_chipset in [self.CHIPSET_BROADCOM, self.CHIPSET_INTEL]:

            # Check Wifi chipset is ready to be set
            self.get_regulatorydomain()

            cmd = "adb shell wpa_cli driver country " + regulatory_domain
            output = self._exec(cmd)
            if "failed" in output.lower():
                msg = "Unable to set regulatory domain - %s" % str(output)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif wifi_chipset == self.CHIPSET_TI:

            cmd = "adb shell iw reg set " + regulatory_domain
            output = self._exec(cmd)
            if "command failed" in output.lower():
                msg = "Unable to set regulatory domain - %s" % str(output)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        else:
            msg = "Unknown Wifi Chipset : %s" % str(wifi_chipset)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Store set value for later restoration.
        self.__last_set_reg_domain = regulatory_domain

        # Control the value set
        value_set = self.get_regulatorydomain()

        if value_set != regulatory_domain:
            msg = "Regulatory domain set fails. Try to set: %s. Read: %s" \
                  % (regulatory_domain, value_set)
            self._logger.warning(msg)

    @need('wifi')
    def get_regulatorydomain(self):
        """
        Get the Wifi Regulatory Domain

        :rtype: String
        :return: the regulatory domain (FR, GB, US... or "none")
        """
        wifi_chipset = self._get_wifi_chipset_manufacturer()

        if wifi_chipset == self.CHIPSET_BROADCOM:

            cmd = "adb shell /system/bin/wlx country"
            output = self._exec(cmd)

            if "/system/bin/wlx: not found" in output:
                msg = "/system/bin/wlx broadcom binary tool is not installed."
                msg += " Regulatory domain check is disabled."
                self._logger.error(msg)

                # Ensure Wifi is enabled
                if self.get_wifi_power_status() != 1:
                    time.sleep(1)
                    self.set_wifi_power(1)

                return "none"

            if "driver adapter not found" in output:
                # Ensure Wifi is enabled
                if self.get_wifi_power_status() != 1:
                    time.sleep(1)
                    self.set_wifi_power(1)

                # And rerun the command
                output = self._exec(cmd)

            refind = re.search(r"^([A-Z0-9]*) \(", output)
            if refind is None or refind.group(1) is None or refind.group(1) == "":
                msg = "Unable to parse regulatory domain: " + output
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            regulatory_domain = refind.group(1)
            if regulatory_domain == "XX":
                regulatory_domain = "none"

        elif wifi_chipset in [self.CHIPSET_TI, self.CHIPSET_INTEL]:
            regulatory_domain = "none"
            cmd = "adb shell iw reg get"
            output = self._exec(cmd)

            refind = re.findall(r"country ([A-Z0-9]*):", output, re.MULTILINE)
            if refind is not []:
                for cur_match in refind:
                    if cur_match not in ["", "00"]:
                        regulatory_domain = cur_match
                        break

            if regulatory_domain == "none":
                msg = "Unable to parse regulatory domain: " + output
                self._logger.error(msg)
                # Temporary workaround due to issue with get reg domain in Intel chips.
                if wifi_chipset != self.CHIPSET_INTEL:
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            msg = "Unknown Wifi Chipset : %s" % str(wifi_chipset)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return regulatory_domain

    @need('wifi')
    def start_wifi_connection_log(self):
        """
        Start logging information about Wifi networks connection
        """
        # Add the message to trig on Logcat
        pattern = "regex:" + SupplicantState.STATE_LOGCAT_FILTER
        self._device_logger.remove_trigger_message(pattern)
        self._device_logger.add_trigger_message(pattern)

    @need('wifi')
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
        # Get the pointer on the list of lines that matches the trigger
        pattern = "regex:" + SupplicantState.STATE_LOGCAT_FILTER
        msg_trigged = self._device_logger.get_message_triggered_status(pattern)

        # Boolean to enable disable log scanning to retrieve supplicant state
        state_scan = False

        # Instanciate the SupplicantState object that will store the latest
        # supplicant state read from the logcat
        state = SupplicantState()

        ret = ""

        # Control the line content
        start_time = time.time()
        while start_time + timeout > time.time():

            # Check if supplicant state has changed within a reasonnable time
            if state.get_state_always_undef() and start_time + (timeout / 5) < time.time():
                # If not, we can consider that the connection failed
                msg = "No supplicant state changes. Network should be out of range"
                self._logger.info(msg)
                ret = "FAILURE"
                break

            if len(msg_trigged) == 0:
                # No new line has been retrieved from logcat, wait for a while
                time.sleep(0.5)
                continue

            # Process the next line
            log_line = msg_trigged[0]
            self._logger.debug("logcat line: " + str(log_line))

            # is log line for starting a new connection on a dedicated SSID?
            ssid_search = re.search(r'SSID=["\']?([^ "\']*)', log_line)

            if ssid_search:
                if ssid.strip() == ssid_search.group(1):
                    # We need to scan supplicant state
                    state_scan = True
                else:
                    # We don't need to scan supplicant state
                    state_scan = False

            elif state_scan:
                # Scan the supplicant state from the log line
                state_read = str(SupplicantState(log_line))

                # Update the supplicant state machine only if the
                # state read is in the next possible states
                if state.set_state(state_read):
                    self._logger.debug("Supplicant state set: " + str(state))

                    if state.is_connection_success():
                        # Connection success
                        msg = "Connection SUCCESS"
                        self._logger.info(msg)
                        ret = "SUCCESS"
                        break

                    if state.is_connection_failure():
                        # All tries fail
                        msg = "MAX connection tries reached (%d)" \
                              % SupplicantState.MAX_CONNECTION_TRIES
                        msg += " -> FAILURE"
                        self._logger.info(msg)
                        ret = "FAILURE"
                        break

            # Remove the 1st line of the list
            del msg_trigged[0]

        # Clean LogCatParser
        self._device_logger.remove_trigger_message(pattern)

        # If a success or a failure has been retrieved, then return it
        if ret:
            return ret

        # If we scanned at least 1 connection failure then we return a failure
        if state.is_connection_failed_once():
            msg = "Connection fails at least once."
            self._logger.info(msg)
            return "FAILURE"

        # No significant connection status has been parsed from the logcat
        self._logger.warning("get_wifi_connection_status_log() timeout")
        return "TIMEOUT"

    @need('wifi')
    def wifi_setkeyexchange(self, ssid, key_exchange_mode, key_exchange_pin=None,
                            simulate_faulty_connection=False, _interface="wlan0"):
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
        cmd = "adb shell wpa_cli -p /data/system/wpa_supplicant wps_cancel"
        self._exec(cmd, timeout=5, wait_for_response=True)
        cmd = "adb shell wpa_cli -p /data/system/wpa_supplicant remove_network %s" % ssid
        self._exec(cmd, timeout=5, wait_for_response=True)

        # Start WPS on DUT.
        cmd = "adb shell wpa_cli -p /data/system/wpa_supplicant %s %s %s" % (wpa_cli_param, bssid, wpa_cli_pin)
        self._logger.debug("Launching supplicant " + cmd)
        results = self._exec(cmd, timeout=5, wait_for_response=True)
        if len(results.split()) < 3:
            raise DeviceException(DeviceException.OPERATION_FAILED, results)
        if results.split()[3] not in ["OK", wpa_cli_pin]:
            raise DeviceException(DeviceException.OPERATION_FAILED, results)
        # Wait for the command to be taken into account into the DUT.
        time.sleep(15)

    @need('wifi')
    def _get_bssid_from_ssid(self, ssid, _interface=None):
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

        cmd = "adb shell wpa_cli -p /data/system/wpa_supplicant scan_result"
        results = self._exec(cmd, timeout=5, wait_for_response=True)
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

    def start_usb_tethering(self, delay=(-1), unplug=False):
        """
        start the tethering connection over USB

        :type delay: int
        :param delay: activate the delayed command.
        :type unplug: boolean
        :param unplug: Unplug USB cable after delayed command in order for the ADB connection not to be lost.
        This option is usefull because USB tethering activation causes a short USB cable disconnection.
        If this parameter is set to True then delay will be force to 10 seconds at least.
        """
        self._set_usb_tethering(True, delay, unplug)

    def stop_usb_tethering(self, delay=(-1), unplug=False):
        """
        stop the tethering connection over USB

        :type delay: int
        :param delay: activate the delayed command.
        :type unplug: boolean
        :param unplug: Unplug USB cable after delayed command in order for the ADB connection not to be lost.
        This option is usefull because USB tethering activation causes a short USB cable disconnection.
        If this parameter is set to True then delay will be force to 10 seconds at least.
        """
        self._set_usb_tethering(False, delay, unplug)

    def _set_usb_tethering(self, enable, delay=(-1), unplug=False):
        """
        start/stop the tethering connection over USB

        :type enable: boolean
        :param enable: True to activate USB Tethering, False to deactivate
        :type delay: int
        :param delay: activate the delayed command.
        :type unplug: boolean
        :param unplug: Unplug USB cable after delayed command in order for the ADB connection not to be lost.
        This option is usefull because USB tethering activation causes a short USB cable disconnection.
        If this parameter is set to True then delay will be force to 10 seconds at least.
        """
        # Control the current state
        if enable == self._get_usb_tethering():
            self._logger.info("USB Tethering already in the requested state")
            return

        # 10 sec minium, otherwise, disconnect_board() won't have time to execute
        if unplug and delay < 10:
            if delay >= 0:
                self._logger.warning("set_usb_tethering: delay set to 10 seconds minimum")
            delay = 10

        target_method = "setUsbTethering"
        cmd_args = "--es switch %s --ei delay %d" % ("on" if enable else "off", delay)
        self._internal_exec_v2(self._tethering_module, target_method, cmd_args, is_system=True)

        if unplug:
            start = time.time()
            # setUsbTethering is planned to run in 10 seconds. We need to be disconnect at this time.
            self._device.disconnect_board()
            reminding_time2wait = delay - (time.time() - start)
            time.sleep(reminding_time2wait + 5)
            self._device.connect_board()
            time.sleep(5)

    def _get_usb_tethering(self):
        """
        Get the USB tethering status

        :return: True if USB tethering is enable, False otherwise
        :rtype: bool

        """
        usb_tethering_status = None
        method = "getUsbTethering"
        output = self._internal_exec_v2(self._tethering_module, method, is_system=True)

        if 'usb_tethering_status' in output:
            usb_tethering_status = str_to_bool_ex(output["usb_tethering_status"])

        if usb_tethering_status is None:
            raise AcsToolException(AcsToolException.PHONE_OUTPUT_ERROR, "get USB Tethering status failed")

        return usb_tethering_status

    def check_usb_tethering_state(self, expected_state, timeout=10):
        """
        Control that the USB Tethering feature is in the expected_state
        :type expected_state: boolean
        :param expected_state: the expected state to control
        :type timeout: int
        :param timeout: optional. Time during which to check the USB tethering status
        """
        usb_tether = self._get_usb_tethering()
        start = time.time()
        while usb_tether != expected_state and start + timeout > time.time():
            time.sleep(2)
            usb_tether = self._get_usb_tethering()
        if usb_tether != expected_state:
            if expected_state:
                msg = "Unable to turn ON USB Tethering"
            else:
                msg = "Unable to turn OFF USB Tethering"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def usb_tether(self, wifi_off, unplug_usb, use_flight_mode):
        """
        push and run a script that will execute the test in adb disconnected mode

        :type wifi_off: int
        :param wifi_off: 1 to turn wifi off during usb tethering

        :type unplug_usb: int
        :param unplug_usb: 1 to unplug usb during usb tethering

        :type use_flight_mode: int
        :param use_flight_mode: 1 to turn on flight mode during usb tethering

        :return: None
        """
        script_name = "/data/usb_tether.sh"
        script_output = "/data/usb_tether.log"

        if wifi_off:
            # case 3: activate tethering, ping AP, turn wifi off
            # ping AP (fail) turn wifi on, ping AP, deactivate
            # tethering
            script_data = \
                """#!/system/bin/sh
sleep 10
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setUsbTethering --es switch on -e opcode %s
sleep 60
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setWifiPower --ei mode 0 -e opcode %s
sleep 30
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setWifiPower --ei mode 1 -e opcode %s
sleep 30
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setUsbTethering --es switch off -e opcode %s
sleep 30
""" \
                % (self._tethering_module, self._generate_key(),
                   self._wifi_module, self._generate_key(),
                   self._wifi_module, self._generate_key(),
                   self._tethering_module, self._generate_key())
        elif unplug_usb:
            # case 2: activate tethering, ping AP, unplug USB
            # ping AP (fail) plug USB, ping AP (fail), deactivate
            # tethering
            script_data = \
                """#!/system/bin/sh
sleep 10
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setUsbTethering --es switch on -e opcode %s
sleep 120
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setUsbTethering --es switch off -e opcode %s
sleep 30
""" \
                % (self._tethering_module, self._generate_key(),
                   self._tethering_module, self._generate_key())
        elif not use_flight_mode:
            # case 1: activate tethering, ping AP, activate flight mode
            # ping AP (fail) deactivate flight mode, ping AP, deactivate
            # tethering
            script_data = \
                """#!/system/bin/sh
sleep 10
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setUsbTethering  --es switch on -e opcode %s
sleep 60
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setFlightMode --ei mode 1 -e opcode %s
sleep 30
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setFlightMode --ei mode 0 -e opcode %s
sleep 30
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setUsbTethering  --es switch off -e opcode %s
sleep 30
""" \
                % (self._tethering_module, self._generate_key(),
                   self._connectivity_module, self._generate_key(),
                   self._connectivity_module, self._generate_key(),
                   self._tethering_module, self._generate_key())
        else:
            # case 0: activate tethering, ping AP, deactivate tethering
            script_data = \
                """#!/system/bin/sh
sleep 10
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setUsbTethering --es switch on -e opcode %s
sleep 60
am broadcast -a intel.intent.action.acs.cmd --es class %s --es method setUsbTethering --es switch off -e opcode %s
sleep 30
""" \
                % (self._tethering_module, self._generate_key(),
                   self._tethering_module, self._generate_key())

        # Copy the script file on DUT file system
        tmp_file = tempfile.NamedTemporaryFile(mode="w+b", delete=False)
        tmp_file.write(script_data)
        tmp_file.flush()
        tmp_file.close()
        cmd = "adb push %s %s" % (tmp_file.name, script_name)
        self._exec(cmd)
        os.unlink(tmp_file.name)

        # Set execution permissions
        cmd = "adb shell chmod 777 %s" % script_name
        self._exec(cmd)

        # Run script detached on DUT
        cmd = "adb shell exec nohup %s > %s && echo -n" \
              % (script_name, script_output)
        self._exec(cmd)

    @need('wifi')
    def set_wifi_frequency_band(self, freq_band, silent_mode=False, _interface="wlan0"):
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
        if not silent_mode:
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
        else:
            self._logger.warning("set_wifi_frequency_band function not implemented for this device")

    @need('wifi')
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

        if not silent_mode:
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
        else:
            self._logger.warning(
                "Get Wifi frequency band: get_wifi_frequency_band function not implemented for this device")

        return "0"

    @need('wifi')
    def get_wifi_connection_rate(self, wifi_interface):
        """
        Get the Wifi connection rate

        :type wifi_interface: str
        :param wifi_interface: WiFi interface name (wlan0)

        :rtype: int
        :return: the connection rate in MByte/s. -1 in case of parsing error
        """
        cmd = "adb shell iw dev %s link" % wifi_interface
        output = self._exec(cmd)

        refind = re.search(r"tx bitrate: ([0-9]*\.?[0-9]+) MBit/s", output)
        if refind is not None:
            self._logger.debug("wifi connection rate parsed: "
                               + str(refind.group(1)))
        if refind is None or refind.group(1) is None or refind.group(1) == "":
            msg = "Unable to parse connection rate"
            self._logger().info(msg)
            return -1

        return int(float(refind.group(1)))

    @need('wifi or modem or ethernet')
    def check_ip_protocol(self, protocol):
        """
        Check IP protocol used

        :type protocol: str
        :param silent_mode: IP protocol selected on the DUT for data registration

        :rtype: String
        :return: The message containing the IP address and the protocol

        """
        network_interface = self._device.get_cellular_network_interface()
        cellular_interface_list = [network_interface, "usb0", "usb1", "rmnet0", "rmnet1", "veth0", "veth1", "inm1",
                                   "inm0"]
        ip_address = ""

        for interface in cellular_interface_list:
            try:
                # If the protocol is IPV6 only
                # We check the device has IPV6 but not IPV4 address
                if protocol == "IPV6":
                    ipv6_address = self.get_interface_ipv6_address(interface)
                    try:
                        ipv4_address = self.get_interface_ipv4_address(interface)
                    except AcsBaseException:
                        self._logger.info("The device doesn't have an IPV4 address")
                    else:
                        raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                              "The device get an IPV4 address on IPV6 only mode %s " % ipv4_address)
                    msg = "The device IPV6 address : %s" % ipv6_address
                elif protocol == "IPV4V6":
                    try:
                        ipv6_address = self.get_interface_ipv6_address(interface)
                        ipv4_address = self.get_interface_ipv4_address(interface)
                    except AcsBaseException:
                        self._logger.info("The device doesn't have an IPV4 and/or IPV6 Address ")
                        raise
                    msg = "The device IPV4 address : %s and IPV6 address : %s" % (ipv4_address, ipv6_address)
                elif protocol == "IPV4":
                    ipv4_address = self.get_interface_ipv4_address(interface)
                    try:
                        ipv6_address = self.get_interface_ipv6_address(interface)
                    except AcsBaseException:
                        self._logger.info("The device doesn't have an IPV6 address.")
                    else:
                        if interface == "veth0":
                            pass
                        else:
                            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                                  "The device get an IPV6 address on IPV4 only mode %s " % ipv6_address)
                    msg = "The device IPV4 address : %s" % ipv4_address
                else:
                    error_msg = "Unknown PROTOCOL type, PROTOCOL type should be IPV4, IPV6, IPV4V6"
                    self._logger.error(error_msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
            except AcsBaseException as e:
                # If there was no IP address, a DeviceException was raised with code DeviceException.OPERATION_FAILED
                # In this case check next address
                if (DeviceException.OPERATION_FAILED in e.get_error_message()):
                    self._logger.debug("No %s address for interface %s" % (protocol, interface))
                else:
                    raise
            else:
                self._logger.info("%s address found for interface %s: %s" % (protocol, interface, ip_address))
                return msg

        msg = "Cellular interface scan failed no %s address found !!!" % protocol
        self._logger.error(msg)
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    @need('wifi or modem or ethernet')
    def check_no_ip_address(self):
        """
        Check that no IP address is given to the DUT (if data is not activated)
        """
        network_interface = self._device.get_cellular_network_interface()
        # We check the device does not have any IPV6 and IPV4 addresses

        cmd_ipv6 = "adb shell busybox ifconfig " + network_interface
        output = self._exec(cmd_ipv6)
        # We retreive the IPV6 address
        ipv6_address = re.search(r"inet6 addr:(.*) ", output)

        # We check that no IPV6 has been retrieved, else we raise an exception
        if ipv6_address is None or ipv6_address is "":
            self._logger.info("The device doesn't have any IPV6 address")

        else:
            error_msg = "DUT has IPV6 address !"
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, error_msg)

        # Check that the DUT has no IPV4 address
        # We retreive the IPV4 address
        cmd_ipv4 = "adb shell busybox ifconfig " + network_interface
        output = self._exec(cmd_ipv4)
        # We retreive the IPV4 address
        ipv4_address = re.search(r"inet addr:([0-9.]+) ", output)

        # We check that no IPV4 has been retrieved, else we raise an exception
        if ipv4_address is None or ipv4_address is "":
            self._logger.info("The device doesn't have any IPV4 address")

        else:
            error_msg = "DUT has an IPV4 address ! "
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, error_msg)

    @need('wifi or modem or ethernet')
    def get_interface_ipv6_all_address(self, interface="", timeout=10):
        """
        Returns all the ipv6 addresses of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type timeout: int
        :param timeout: time to wait until exit with error

        :rtype: list of strings
        :return: interface ip addresses
        """
        start = time.time()
        while time.time() < start + timeout:
            cmd = "adb shell /system/bin/ip -6 addr show " + interface
            output = self._exec(cmd)
            all_addr = re.findall(r"inet6\s+(.+)\s+scope", output)
            if all_addr is not None:
                break
            else:
                time.sleep(1)
        if all_addr is None:
            msg = "Could not get IPv6 address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            interface_ips = []
            for cur_addr in all_addr:
                interface_ips.append(re.sub('/[0-9]*$', '', cur_addr[0]))
            if not interface_ips:
                msg = "No matching IPv6 address in the IP address list of %s" % interface
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        return interface_ips

    @need('wifi or modem or ethernet')
    def get_interface_ipv6_scopelink_address(self, interface="", timeout=10):
        """
        Returns the scope link ipv6 address of the given interface.
        .. note:: This function returns only the 1st address of the IPv6 link,

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type timeout: int
        :param timeout: time to wait until exit with error

        :rtype: str
        :return: interface ip address
        """
        start = time.time()
        while time.time() < start + timeout:
            cmd = "adb shell /system/bin/ip -6 addr show " + interface
            output = self._exec(cmd)
            # TODO: This regular expression can just retrieve the ipv6 address
            # We are not sure it's a valid ipV6 address
            m = re.search(r"inet6\s+(.+)+/64\s+scope link", output)
            if m is not None:
                break
            else:
                time.sleep(1)

        if m is None:
            msg = "Could not get IPV6 address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        if len(m.groups()) > 1:
            msg = "DUT has more than on local IPV6 address on interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        else:
            interface_ipv6_local = m.groups()[0]
            return interface_ipv6_local

    @need('wifi or modem or ethernet')
    def get_interface_ipv6_global_address(self, interface="", timeout=10):
        """
        Returns the scope link ipv6 address of the given interface.
        .. note:: This function returns only the 1st address of the IPv6 link,

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type timeout: int
        :param timeout: time to wait until exit with error

        :rtype: str
        :return: interface ip address
        """
        start = time.time()
        while time.time() < start + timeout:
            cmd = "adb shell /system/bin/ip -6 addr show " + interface
            output = self._exec(cmd)
            # TODO: This regular expression can just retreive the ipv6 address
            # We are not sure it's a valid ipV6 address
            m = re.search(r"inet6\s+(.+)\s+scope global", output)
            if m is not None:
                break
            else:
                time.sleep(1)

        if m is None:
            msg = "Could not get IPV6 address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if len(m.groups()) > 1:
            msg = "DUT has more than on local IPV6 address on interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        else:
            interface_ipv6_global = m.groups()[0]
            return interface_ipv6_global

    @need('wifi or modem or ethernet')
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
        result = True
        dut_mac_address = self.get_interface_mac_addr().upper()
        dut_ipv6_addresses = self.get_interface_ipv6_all_address(interface)
        loop_prefix = "FE80"
        network_ip_found = False
        loop_ip_found = False

        # Wait for the local IP to be acquired
        tries = 5
        while (len(dut_ipv6_addresses) < 2) and (tries > 0):
            time.sleep(2)
            dut_ipv6_addresses = self.get_interface_ipv6_all_address(interface)
            tries -= 1

        if ipv6_prefix == "":
            # Don't check that.
            network_ip_found = True

        for cur_ip in dut_ipv6_addresses:
            if cur_ip.upper().startswith(loop_prefix + ':'):
                loop_ip_found = True
                dut_ipv6_addr = cur_ip.upper()
                break
            elif ((ipv6_prefix != "") and
                      (cur_ip.upper().startswith(ipv6_prefix.upper()))):
                network_ip_found = True

        if not loop_ip_found:
            result = False
        else:
            if not network_ip_found:
                msg = "IPV6 addresses consistency check failed "
                msg += "(loopback IPv6 OK, but no network IPv6 found)!"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            # Format prefix:M1M2:M3ff:feM4:M5M6
            # M1 2nd bit should be reverted
            expected_ipv6_ip = "%s::%X%s:%s:%s:%s" % (loop_prefix,
                                                      (int(dut_mac_address[0:2], 16) ^ 2),
                                                      dut_mac_address[3:5],
                                                      dut_mac_address[6:8] + "FF",
                                                      "FE" + dut_mac_address[9:11],
                                                      dut_mac_address[12:14] + dut_mac_address[15:17]
                                                      )
            expected_ipv6_ip = expected_ipv6_ip.upper()

            if dut_ipv6_addr != expected_ipv6_ip:
                msg = "IPV6 addresses consistency check failed "
                msg += "(DUT IPv6 %s is not the expected one %s!" % \
                       (dut_ipv6_addr, expected_ipv6_ip)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                msg = "IPV6 addresses consistency check OK."
                self._logger.info(msg)

        return result

    @need('wifi or modem or ethernet')
    def launch_dhcp6_client(self, dhcp6_mode, interface):
        """
        Launched the ipv6 address of the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :type dhcp6_mode: str
        :param dhcp6_mode: stateless or stateful
        """
        mode_to_set = dhcp6_mode.strip().lower()
        interface = interface.lower()
        if mode_to_set not in ['stateless', 'stateful']:
            msg = "Invalid DHCP6 mode %s" % dhcp6_mode
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        self._logger.info("Starting DHCP6 client on DUT...")
        dhcp6c_working = False

        # Try to kill and ask again for a dynamic IP 5 times.
        tries_launch_client = 5
        while (dhcp6c_working == False) and (tries_launch_client > 0):
            cmd = "adb shell killall -s KILL dhcp6c"
            self._exec(cmd, wait_for_response=False)
            time.sleep(3)
            cmd = "adb shell dhcp6c -c /etc/dhcp6c/dhcp6c.%s.%s.conf %s" % \
                  (interface, mode_to_set, interface)
            self._exec(cmd, wait_for_response=False)

            # Ask if dynamic IP has been acquired with a 10s timeout.
            tries_get_ip = 10
            while (dhcp6c_working == False) and (tries_get_ip > 0):
                cmd = "adb shell ip -6 addr show scope global %s" % interface
                output = self._exec(cmd, wait_for_response=True)
                if interface in output:
                    dhcp6c_working = True
                    break
                else:
                    tries_get_ip -= 1
                    time.sleep(1)
            tries_launch_client -= 1

        if dhcp6c_working:
            self._logger.info("Started DHCP6 client on DUT.")
        else:
            msg = "Can't acquire IPV6 address with DHCP6 client on DUT"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def launch_rdnssd_client(self, interface):
        """
        Launches the Recursive DNS Server on the given interface.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        self._logger.info("Starting RDNSSD client on DUT...")

        cmd = "adb shell rdnssd -i %s" % (interface.lower())
        self._exec(cmd, wait_for_response=False)

        self._logger.info("Started RDNSSD client on DUT.")

    def _get_dhcp6_dns(self, interface, dns_nr=1, is_rdnssd=False):
        """
        Gets the DHCP6 DNS of the given interface.

        :type interface: String
        :param interface: interface name (wlan0/wlan1 etc...)

        :type dns_nr: int
        :param dns_nr: dns server number

        :type is_rdnssd: Boolean
        :param is_rdnssd: If true, the dns will be taken via the rdnssd property,
        If False, it will be done via DHCP.

        :rtype: str
        :return: The IPV6 address of the DNS server.
        """

        if is_rdnssd is True:
            dns_mode = "rdnssd"
        else:
            dns_mode = "dhcp6"
        dns_id = str(dns_nr)

        result = self._phone_system.get_property_value("%s.%s.dns%s"
                                                       % (dns_mode, interface.lower(), dns_id))
        return result

    def _set_net_dhcp6_dns(self, interface, dns_nr, value):
        """
        Sets the net DNS  property of the given interface.

        :type interface: String
        :param interface: interface name (wlan0/wlan1 etc...)

        :type dns_nr: int
        :param dns_nr: dns server number

        :type value: String
        :param value: DNS IPV6 address.
        """
        dns_id = str(dns_nr)
        cmd = "adb shell setprop net.%s.dns%s %s" % (interface.lower(), dns_id, value)
        self._exec(cmd, wait_for_response=True)
        cmd = "adb shell setprop net.dns%s %s" % (dns_id, value)
        self._exec(cmd, wait_for_response=True)

    def copy_dhcp6_dns(self, interface='wlan0', dns_nr='1', is_rdnssd=False):
        """

        Copy the DNS of the given interface from dhcp6.<interface>.dns to
        net.<interface>.dns

        :type interface: String
        :param interface: interface name (wlan0/wlan1 etc...)

        :type dns_nr: int
        :param dns_nr: dns server number.

        :type is_rdnssd: Boolean
        :param is_rdnssd: If true, the dns will be taken via the rdnssd property,
        If False, it will be done via DHCP.
        """
        value_to_set = self._get_dhcp6_dns(interface, dns_nr, is_rdnssd)
        self._set_net_dhcp6_dns(interface, dns_nr, value_to_set)

    @need('wifi')
    def measure_wifi_turns_off_duration(self, mode, wlaninterface,
                                        duration=(-1), period=(-1),
                                        expected_duration=(-1), tolerance=(-1)):
        """
        Measure the time for WiFi interface to turn OFF.
        For test that should be performed unplugged, this function can be used
        with asynchronous mode.

        :type mode: str
        :param mode: Can be "sync", "async_init", "async_result"
        - "sync" -> Synchrone mode.
            The time to turn off WiFi is returned by the function
            An exception is raised measured value too far from
            expected_duration passed as parameter
        - "async_init" -> Asynchronous mode, intialisation.
            An embedded script to watch WiFi interface activation is launched
            on the DUT.
        - "async_result" -> Asynchronous mode, obtains the result
            Retrieves the result from the DUT and returns it.
            An exception is raised measured value too far from
            expected_duration passed as parameter
        :type wlaninterface: str
        :param wlaninterface: Wireless LAN interface of the DUT (ie: wlan0)
        :type duration: int
        :param duration: duration time while watching the WiFi interface (in s)
        :type period: int
        :param period: waiting time between occurences (in s)
                default is automatic value (duration / 100)
        :type expected_duration: int
        :param expected_duration: Expected time for WiFi to turn OFF in sec.
        :type tolerance: float
        :param tolerance: tolerance for expected_duration in percentage < 1

        :rtype: int or None
        :return: with sync and async_result modes, returns the time for WiFi to
            turn OFF in seconds. It returns None in async_init mode.
        """
        if mode == "sync":
            if duration == -1 or expected_duration == -1 or tolerance == -1:
                msg = "duration, expected_duration and expected_duration " + \
                      "parameters should be specified in synchronous mode"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        elif mode == "async_init":
            if duration == -1:
                msg = "duration parameter should be specified in " + \
                      "init phase of asynchronous mode"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        elif mode == "async_result":
            if expected_duration == -1 or tolerance == -1:
                msg = "expected_duration and tolerance parameters " + \
                      "should be specified in result phase of asynchronous mode"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        else:
            msg = "mode is unavailable: " + mode
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        output_file = "/data/acs_watch_wifi_interface.log"

        # Script start up
        if mode in ["sync", "async_init"]:
            script_name = "/data/acs_watch_wifi_interface.sh"
            script_data = \
                """#!/system/bin/sh
DURATION=10
PERIOD=1
OUTFILE=""
TIMESTAMP_TYPE="r"

function usage()
{
    echo "Watch WiFi interface activation"
    echo ""
    echo "Usage: $0 [-t a|r] [-w <outfile>] [-d <duration>] [-p <period>]"
    echo ""
    echo " -t a|r \tTimestamp type: absolute or relative (default $TIMESTAMP_TYPE)"
    echo " -w <outfile> \tSet the output filename (default stdout)"
    echo " -d <duration> \tDuration of the watch in seconds (default infinite)"
    echo " -p <period> \tPeriod for WiFi interface test in seconds (default $PERIOD)"
    echo ""
}

# Parse script parameters
while [ "$1" != "" ]; do
    PARAM=$1
    VALUE=$2
    case $PARAM in
    -h | --help)
        usage
        exit
        ;;
    -t)
        TIMESTAMP_TYPE=$VALUE
        ;;
    -w)
        OUTFILE=$VALUE
        ;;
    -d)
        DURATION=$VALUE
        ;;
    -p)
        PERIOD=$VALUE
        ;;
    *)
        echo "ERROR: unknown parameter \"$PARAM\""
        usage
        exit 1
        ;;
    esac
    shift
    shift
done

echo duration: $DURATION
echo period: $PERIOD
echo outfile: $OUTFILE
echo timestamp type: $TIMESTAMP_TYPE

# Empty the output file
if [ "$OUTFILE" != "" ]; then
    if [ -f $OUTFILE ]; then
        rm -f $OUTFILE
    fi
fi

STARTTIME=`date +%%s`
CURRENT=$STARTTIME
ELAPSED=0
while [ `expr $ELAPSED \< $DURATION` == "1" ]
do
        # Calculate the timestamp
    if [ $TIMESTAMP_TYPE == "a" ]; then
        TS=$CURRENT
    else
        TS=$ELAPSED
    fi

    # Check WiFi interface activation
    iswifi=`busybox ifconfig | grep "%s  "`

    # Print the output result
    if [ "$iswifi" ]
    then
        if [ "$OUTFILE" ]; then
        echo "$TS:ON" >> $OUTFILE
        else
        echo "$TS:ON"
        fi
    else
        if [ "$OUTFILE" ]; then
        echo "$TS:OFF" >> $OUTFILE
        else
        echo "$TS:OFF"
        fi
    fi
    sleep $PERIOD
    CURRENT=`date +%%s`
    ELAPSED=`expr $CURRENT - $STARTTIME`
done""" % wlaninterface
            if period <= 0:
                period = int(duration / 100)
                if period <= 0:
                    period = 1

            # Copy the script file on DUT file system
            tmp_file = tempfile.NamedTemporaryFile(mode="w+b", delete=False)
            tmp_file.write(script_data)
            tmp_file.flush()
            tmp_file.close()
            cmd = "adb push %s %s" % (tmp_file.name, script_name)
            self._exec(cmd)
            os.unlink(tmp_file.name)

            # Set execution permissions
            cmd = "adb shell chmod 777 %s" % script_name
            self._exec(cmd)

            # Run script detached on DUT
            if mode == "async_init":
                cmd = "adb shell exec nohup %s -w %s -d %d -p %d > /dev/null && echo -n" \
                      % (script_name, output_file, duration, period)
                self._exec(cmd)
            else:
                # Synchronous mode
                cmd = "adb shell sh %s -d %d -p %d" \
                      % (script_name, duration, period)
                result = self._exec(cmd, timeout=duration + 10)
                self._logger.debug("synchronous script result: " + result)
                # Parse the time WiFi goes OFF
                regexp_search = re.search(r'([0-9]+):OFF', result)
                if regexp_search:
                    result = regexp_search.group(1)
                else:
                    result = "No response from ADB"

        elif mode == "async_result":
            # retrieve the result from the DUT
            cmd = "adb shell grep 'OFF' %s | head -n 1 | cut -d ':' -f 1" % output_file
            result = self._exec(cmd, timeout=5)

        if mode in ["sync", "async_result"]:
            # error handling
            if "No response from ADB" in result:
                msg = "WiFi never turns OFF. expected [%d]" % expected_duration
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if not str(result).isdigit():
                msg = "Unable to parse watch_wifi_interface_report: "
                msg += result
                self._logger.error(msg)
                raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, msg)

            # Compare the measure with the expected value
            result = int(result)
            if result < expected_duration * (1 - tolerance):
                msg = "Wifi turns OFF earlier than expected: %d [exp: %d]" \
                      % (result, expected_duration)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if result > expected_duration * (1 + tolerance):
                msg = "Wifi turns OFF later than expected: %d [exp: %d]" \
                      % (result, expected_duration)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            self._logger.debug("measured time for WiFi to turn OFF: %d sec" %
                               result)
            return result

        return None

    @need('wifi or modem or ethernet')
    def get_interface_ipv6_address(self, interface):
        """
        Returns the 1st ipv6 address of the given interface.
        .. note:: This function returns only the 1st address of the IPv6 link,
        get_interface_ipv6_all_address will return all the addresses.

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: str
        :return: interface ip address
        """
        cmd = "adb shell busybox ifconfig " + interface
        output = self._exec(cmd)
        # TODO: This regular expression can just retreive the ipv6 address
        # We are not sure it's a valid ipV6 address
        m = re.search(r"inet6 addr:(.*) ", output)
        if m is None:
            msg = "Could not get IPV6 address of interface %s" % interface
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        interface_ip = m.groups()[0]
        return interface_ip

    @need('modem')
    def get_roaming_mode(self):
        """
        Returns the roaming mode.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        method = "getRoamingMode"
        output = self._internal_exec_v2(self._cellular_networking_module, method, is_system=True)

        return int(output[self.OUTPUT_MARKER])

    @need('modem')
    def set_roaming_mode(self, mode):
        """
        Sets the roaming mode to off or on.

        :type mode: str
        :param mode: can be 'on' to enable
                            'off' to disable

        :return: None
        """
        method = "setRoamingMode"
        current_mode = str(self.get_roaming_mode())
        if mode.lower() == "on":
            if current_mode == mode:
                warning_msg = "roaming mode is already enabled"
                self._logger.info(warning_msg)
            else:
                mode = "1"
                self._logger.info("Enabling roaming mode ...")
                arg = "--ei mode %s" % mode
                self._internal_exec_v2(self._cellular_networking_module, method, arg, is_system=True)

        elif mode.lower() == "off":
            if current_mode == mode:
                warning_msg = "roaming mode is already disabled"
                self._logger.info(warning_msg)
            else:
                mode = "0"
                self._logger.info("Disabling roaming mode ...")
                arg = "--ei mode %s" % mode
                self._internal_exec_v2(self._cellular_networking_module, method, arg, is_system=True)
        else:
            error_msg = \
                "set_roaming_mode : Parameter mode %s is not valid" % mode
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

    @need('wifi or modem or ethernet')
    def ftpput(self, server_ip_address, username, password, remote_path,
               local_path):
        """
        Launches a ftpput on the device.
        If the username and password parameters are empty the uecommand will
        try to upload the file using anonymous login.
        :type server_ip_address: str
        :param server_ip_address: Ip address to connect to in order to make
        the transfer.
        :type username: str
        :param username: username to use to connect to the ftp server.
        :type password: str
        :param password: password to use to connect to the ftp server.
        :type remote_path: str
        :param remote_path: path to the remote file to write to the ftp server.
        :type local_path: str
        :param local_path: path to the file on the device to upload to the ftp
        server.
        :rtype: process
        :return: process where the ftp has been launched.
        """
        # Check if the directory structure from 'local_path' is valid
        local_dir = os.path.dirname(local_path)
        if self._phone_system.check_directory_exist_from_shell(local_dir) == False:
            self._logger.info("Local path is not valid, create it")
            self._phone_system.create_directory_from_shell(local_dir)

        # Preliminary analysis : create file to upload if not
        # already present in the device (IF POSSIBLE)
        try:
            size_kb = self.__file_uecmds.retrieve_size_from_filename(local_path)
            self.__file_uecmds.create_file_if_needed(local_path, size_kb, os.path.dirname(local_path))
        except AcsBaseException as excp:
            self._logger.info(
                    "Preliminary analysis failed : we won't be able to "
                    "create file if not already present in the device")
            self._logger.debug("Root cause : " + str(excp))

        # On CMW500, anonymous FTP is also supported, FTP service isn't stable
        # we have always the login issue with correct login & password
        if username is "" and password is "":
            # If username and password are empty use anonymous login.
            self._logger.warning("Using anonymous login to log on ftp server.")
            ftp_cmd = "adb shell ftpput %s %s %s" % (server_ip_address,
                                                     remote_path,
                                                     local_path)
        elif (username is "" and password is not "") or \
                (username is not "" and password is ""):
            # If one of password and username is empty while the other is not.
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Both username and password for the ftp "
                                     "server can be empty but one of them can "
                                     "not be empty while the other is not.")
        else:
            ftp_cmd = "adb shell ftpput -u %s -p %s %s %s %s" \
                      % (username,
                         password,
                         server_ip_address,
                         remote_path,
                         local_path)
        self._logger.info("The FTP command : %s" % ftp_cmd)
        # Start ftp transfers in another thread
        (process, _q) = run_local_command(ftp_cmd)
        return process, _q

    @need('wifi or modem or ethernet')
    def ftpget(self, server_ip_address, username, password, local_path,
               remote_path):
        """
        Launches a ftpget on the device.
        If the username and password parameters are empty the uecommand will
        try to upload the file using anonymous login.
        :type server_ip_address: str
        :param server_ip_address: Ip address to connect to in order to make
        the transfer.
        :type username: str
        :param username: username to use to connect to the ftp server.
        :type password: str
        :param password: password to use to connect to the ftp server.
        :type remote_path: str
        :param remote_path: path to the remote file to download from the
        ftp server.
        :type local_path: str
        :param local_path: path where to write the downloaded file on the
        devices.
        :rtype: process
        :return: process where the ftp has been launched.
        """
        # Check if the directory structure from 'local_path' is valid
        local_dir = os.path.dirname(local_path)
        if self._phone_system.check_directory_exist_from_shell(local_dir) == False:
            self._logger.info("Local path is not valid, create it")
            self._phone_system.create_directory_from_shell(local_dir)

        if username is "" and password is "":
            # If username and password are empty use anonymous login.
            self._logger.warning("Using anonymous login to log on ftp server.")
            ftp_cmd = "adb shell ftpget %s %s %s" % (server_ip_address,
                                                     local_path,
                                                     remote_path)
        elif (username is "" and password is not "") or \
                (username is not "" and password is ""):
            # If one of password and username is empty while the other is not.
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Both username and password for the ftp "
                                     "server can be empty but one of them can "
                                     "not be empty while the other is not.")
        else:
            ftp_cmd = "adb shell ftpget -u %s -p %s %s %s %s" \
                      % (username,
                         password,
                         server_ip_address,
                         local_path,
                         remote_path)
        self._logger.info("The FTP command : %s" % ftp_cmd)
        # Start ftp transfers in another thread
        (process, _q) = run_local_command(ftp_cmd)
        return process, _q

    @need('modem')
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
        """
        method = "getPreferredNetworkType"
        net_type = self._internal_exec_v2(self.network_type_module, method, is_system=True)
        net_type_value = net_type.get("networkTypeValue", None)
        net_type_name = net_type.get("networkTypeName", None)
        if net_type is None:
            error_msg = \
                "getPreferredNetworkType : Parameter network value %s is not valid" % net_type_value
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        else:
            msg = \
                "Actual network type is %s and it's value is %s  " % (net_type_name, str(net_type_value))
            self._logger.info(msg)
        return self.NETWORK_TYPE_CONVERSION_TABLE[int(net_type_value)]

    @need('modem')
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
        :return: None
        """
        self._logger.info(
                "Setting preferred network type on dut ...")
        current_type = self.get_preferred_network_type()
        if current_type == preferred_type:
            warning_msg = "the preferred network '%s' type is already enabled" % str(preferred_type)
            self._logger.info(warning_msg)
        else:
            preferred_type = get_dict_key_from_value(self.NETWORK_TYPE_CONVERSION_TABLE, preferred_type)
            self._logger.info("Setting the wanted preferred network type: '%s' " % str(preferred_type))
            method = "setPreferredNetworkType"
            cmd = " --ei networkType %s" \
                  % str(preferred_type)
            self._internal_exec_v2(self.network_type_module, method, cmd, is_system=True)

    @need('wifi')
    def _get_wifi_chipset_manufacturer(self):
        """
        Get Wifi chipset manufacturer: CHIPSET_BROADCOM, CHIPSET_INTEL or CHIPSET_TI
        List of possible values are attributes of this class

        :rtype: str
        :return: The Wifi chipset used on the PHONE
        """
        if self._wifi_chipset_manufacturer is None:
            cmd = 'adb shell lsmod'
            out = self._exec(cmd, force_execution=True, timeout=10)
            out = out.lower()
            if re.search(r'^wl12xx', out, re.MULTILINE) is not None:
                self._logger.debug("Wifi Chipset TI detected")
                self._wifi_chipset_manufacturer = self.CHIPSET_TI
            elif re.search(r'^iwlwifi', out, re.MULTILINE) is not None:
                self._logger.debug("Wifi Chipset Intel detected")
                self._wifi_chipset_manufacturer = self.CHIPSET_INTEL
            elif re.search(r'^bcm', out, re.MULTILINE) is not None:
                self._logger.debug("Wifi Chipset Broadcom detected")
                self._wifi_chipset_manufacturer = self.CHIPSET_BROADCOM
            else:
                self._logger.debug("Unable to detect Wifi Chipset.")
                self._wifi_chipset_manufacturer = self.CHIPSET_UNKNOWN

        return self._wifi_chipset_manufacturer

    def disable_output_traffic(self):
        """
        Prevent all traffic including icmp

        :param: None
        :return: None
        """
        self._logger.info("Disable all output traffic")
        cmd = "adb shell iptables -I OUTPUT 1 -p all -j DROP"
        self._exec(cmd, 10)

    def enable_output_traffic(self):
        """
        Allow UE traffic to go out

        :param: None
        :return: None
        """
        self._logger.info("Enable all output traffic")
        cmd = "adb shell iptables -D OUTPUT 1"
        self._exec(cmd, 10)

    def disable_output_traffic_but_ping(self):
        """
        Prevent all traffic excluding ping

        :param: None
        :return: None
        """
        self._logger.info("Disable output traffic except ping")
        cmd = "adb shell iptables -I OUTPUT 1 ! -p 1 -j DROP"
        self._exec(cmd, 10)

    def get_default_regulatory_domain(self):
        """
        Get default regulatory domain

        :rtype: str
        :return: The default regulatory domain
        """
        return self.DEFAULT_REG_DOMAIN

    @need('modem')
    def get_serving_cell(self):
        """
        Returns the current serving cell of the device.

        :rtype: acs_test_scripts.Utilities.RegistrationUtilities.CellInfo
        :return: the serving cell instance
        """
        # Call the Android UE Command
        method = "getServingCellInfo"
        info = self._internal_exec_v2(self.network_type_module, method, is_system=True)
        # Build a cell info instance
        serving_cell = CellInfo.from_dict(info)
        # Return the instance
        return serving_cell

    @need('modem')
    def get_neighboring_cells(self):
        """
        Returns the list of all neighboring cells.

        :rtype: list
        :return: a list of C{NeighboringCellInfo} instances
        """
        # Create a list to record all neighboring cells
        neighboring_cells = []
        # Call the Android UE Command
        method = "getNeighboringCellInfo"
        cell_descriptions = self._internal_exec_multiple_v2(
                self.network_type_module,
                method, is_system=True)
        # Iterate on results
        for cell_description in cell_descriptions:
            # Let the base class instantiate the
            # correct classes for us.
            neighbor = CellInfo.from_dict(cell_description)
            neighboring_cells.append(neighbor)
        # Return the list
        return neighboring_cells

    def simulate_phone_pin_unlock(self, pin_lock_code=""):
        """
        Simulates the entry of the PHONE pin code by the user.
        This is intended to unlock the certificate for enterprise authentication if
        it protected by one.

        :type pin_lock_code: String
        :param pin_lock_code: The code used to lock the phone
        """
        KEYCODE_ENTER = "66"
        if pin_lock_code != "":
            if pin_lock_code.isdigit():
                self._logger.debug("Simulating phone PIN unlock")
                self._phone_system.display_off()
                time.sleep(0.5)
                self._phone_system.display_on()
                self._exec("adb shell input text " + pin_lock_code)
                self._exec("adb shell input keyevent " + KEYCODE_ENTER)
            else:
                error_msg = "PIN code must be a digit."
                self._logger.error(error_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        else:
            self._logger.warning("simulate_phone_pin_unlock: No PIN code provided. Don't do anything")

    @need('wifi')
    def _get_wifi_security_type(self, security_parameter):
        """
        Get the Android security type to set on device from the given security.

        :type security_parameter : str
        :param security_parameter: "normalized" security name. Can be NONE, OPEN, WEP, WPA, WPA2,
         WEP64-OPEN, WEP128-OPEN, WEP64, WEP128, WPA2-PSK-AES, WPA-PSK-TKIP,
         WPA2-PSK-TKIP, WPA-PSK-AES, WPA-PSK, WPA-PSK-TKIP-AES,
         WPA2-PSK, WPA2-PSK-TKIP-AES, WPA-WPA2-PSK, WPA-WPA2-PSK-TKIP-AES,
         WPA-WPA2-PSK-TKIP, "WPA-WPA2-PSK-AES

        :rtype: str
        :return: Android specific security name
        """
        # Extract the security Type
        security_upper = str(security_parameter).upper()
        if security_upper in ("NONE", "OPEN"):
            security = "NONE"
        elif security_upper in ("WEP", "WEP64", "WEP128"):
            security = "WEP"
        elif security_upper in ("WEP64-OPEN", "WEP128-OPEN"):
            security = "WEP-OPEN"
        elif security_upper in ("WPA", "WPA2", "WPA2-PSK-AES", "WPA-PSK-TKIP",
                                "WPA2-PSK-TKIP", "WPA-PSK-AES",
                                "WPA-PSK", "WPA-PSK-TKIP-AES",
                                "WPA2-PSK", "WPA2-PSK-TKIP-AES",
                                "WPA-WPA2-PSK", "WPA-WPA2-PSK-TKIP-AES",
                                "WPA-WPA2-PSK-TKIP", "WPA-WPA2-PSK-AES"):
            security = "WPA"
        elif security_upper in ("EAP-WPA", "EAP-WPA2", "EAP-WPA-WPA2"):
            security = security_upper
        else:
            msg = "Unexpected Wlan security %s!" % str(security_upper)
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

        return security

    @need('wifi')
    def get_wifi_rssi(self, ssid):
        """
        Returns the WiFi RSSI for a given SSID.

        :type ssid : str
        :param ssid: name of SSID to check

        :rtype: int
        :return: value of the RSSI in dBm
        """

        method = "getWifiRSSI"
        cmd_args = "--es SSID %s" % ssid
        output = self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

        status = output.get("wifi_rssi", None)
        self._logger.debug("RSSI VALUE = %sdBm" % status)

        return int(status)

    @need('wifi')
    def set_network_notification(self, mode):
        """
        Set the Network Notification option. Available in Advanced Wi-Fi settings menu.

        :type mode: string or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """

        notification_mode = str_to_bool_ex(mode)
        if notification_mode is None:
            self._logger.error("set_network_notification : Parameter mode %s is not valid" % str(mode))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter 'mode' is not valid !")
        elif notification_mode is True:
            self._logger.info("Setting network notification to on")
            mode = "1"
        elif notification_mode is False:
            self._logger.info("Setting network notification to off")
            mode = "0"

        method = "setNetworkNotification"
        cmd_args = "--ei mode %s" % mode
        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

    def get_network_notification(self):
        """
        Get the Network Notification option. Available in Advanced Wi-Fi settings menu.

        @rtype: boolean
        @return: state of network notification (0 for disabled, 1 for enabled)
        """
        method = "getNetworkNotification"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)

        status = output.get("wifi_network_notifications", None)
        self._logger.debug("STATUS VALUE = %s" % status)

        if status in ["0", "1"]:
            return int(status)
        else:
            msg = "Invalid return value of getNetworkNotification : %s" % status
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def clean_tcp_records_device(self):
        """
        Clean TCP records on Board and remote computer
        """

        cmd = "adb shell sysctl -w net.ipv4.route.flush=1"
        self._exec(cmd)

    def cellular_signal_properties(self):
        '''
           Returns a dictionary of properties about the cellular signal.
        '''
        adb_cmd = 'adb shell dumpsys telephony.registry | grep mSignalStrength'
        (return_code, dumpsys_output) = self._device.run_cmd(adb_cmd, 10)
        # The dumpsys_output should be a str of numbers separated by spaces like this:
        #  mSignalStrength=SignalStrength: 99 -1 -120 -160 -120 -1 -1 99 -114 -12 60 2147483647 gsm|lte
        strength_props = dumpsys_output.split()
        if len(strength_props) < 14:
            raise Exception(
                "Unknown output from command 'dumpsys telephony.registry | grep mSignalStrength'. Expected 13 items separated by whitespace.")
        return_dict = dict()
        return_dict['GSM Signal Strength'] = int(strength_props[1])
        return_dict['GSM Bit Error Rate'] = int(strength_props[2])
        return_dict['CDMA dBm'] = int(strength_props[3])
        return_dict['CDMA Ec/Io'] = int(strength_props[4])
        return_dict['EVDO dBm'] = int(strength_props[5])
        return_dict['EVDO Ec/Io'] = int(strength_props[6])
        return_dict['EVDO SNR'] = int(strength_props[7])
        return_dict['LTE Signal Strength'] = int(strength_props[8])
        return_dict['LTE RSRP'] = int(strength_props[9])
        return_dict['LTE RSRQ'] = int(strength_props[10])
        return_dict['LTE RSSNR'] = int(strength_props[11])
        return_dict['LTE CQI'] = int(strength_props[12])
        return_dict['Connection Type'] = strength_props[13]
        return return_dict

    def start_multiple_http_transfer(self, url, sync_interval, duration, agent="acs", filename="test_dl.data"):
        """
        Start http download to a given address every X millis during X seconds

        :type url: str
        :param url: the url to the file to download

        :type sync_interval: int
        :param sync_interval: the download interval

        :type duration: int
        :param duration: the duration until transfers will stop in seconds

        :type filename: str
        :param filename: optional parameter to specified the filename of downloaded file

        :type agent: str
        :param agent: agent to use to process the download

        :return: status & output
        """
        if agent not in ["acs", "SPMActiveIdle"]:
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR,
                                     "unknown agent '{0}' to process download".format(agent))

        status = "FAILURE"
        output_msg = ""
        cmd = "adb shell [ -d {0} ] || mkdir -p {0}".format(self._device.multimedia_path)
        self._exec(cmd)

        if agent == "acs":
            download_file_path = self._device.get_device_os_path().join(self._device.multimedia_path,
                                                                        filename)
            # convert seconds as millis
            duration = duration * 1000
            method = "startHttpTransferts"
            cmd_args = "--es url {0} --es file_path {1} --ei sync_interval {2} --ei duration {3}".format(url,
                                                                                                         download_file_path,
                                                                                                         sync_interval,
                                                                                                         duration)
            output = self._internal_exec_v2(self._http_transfer_module, method, cmd_args, is_system=True)
            status, output_msg = output[self.STATUS_MARKER], output[self.OUTPUT_MARKER]
        else:
            regex_download_count = "regex:.*SPMActiveIdle: {0}.*".format(url)
            regex_download_ok = "regex:.*SPMActiveIdle: Download complete.*"
            self._device_logger.add_trigger_message(regex_download_count)
            self._device_logger.add_trigger_message(regex_download_ok)
            cmd_args = "--es URL '{0}' --ei SYNC_INTERVAL {1} --ei TIME {2}".format(url,
                                                                                    sync_interval / 1000,
                                                                                    duration)
            cmd = "adb shell am start -n com.intel.spm.SPMActiveIdle/.SPMActiveIdle {0}".format(cmd_args)
            output_msg = self._exec(cmd)
            status = "SUCCESS"
        return status, output_msg

    def get_status_multiple_http_transfer(self):
        """
        Get status of transfers

        :return: downloading status, transfers count and transfers error
        """
        method = "getTransferStatus"
        output = self._internal_exec_v2(self._http_transfer_module, method, is_system=True)

        status = output.get("is_downloading", None)
        self._logger.debug("STATUS VALUE = %s" % status)

        if status is not None:
            return str_to_bool_ex(status), int(output.get("download_count", 0)), int(output.get("error_count", 0))
        else:
            msg = "Invalid return value of getTransferStatus : %s" % status
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def stop_multiple_http_transfer(self, url, agent="acs"):
        """
        Stop the http transfers

        :type url: str
        :param url: the url of the ongoing transfer

        :type agent: str
        :param agent: agent to use to process the download

        :return: status, transfers count and transfers error
        """
        if agent not in ["acs", "SPMActiveIdle"]:
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR,
                                     "unknown agent '{0}' to process download".format(agent))
        status = "FAILURE"
        total_download = 0
        total_error = 0

        if agent == "acs":
            method = "stopHttpTransfers"
            output = self._internal_exec_v2(self._http_transfer_module, method, is_system=True)
            status = output[self.STATUS_MARKER]
            total_download = int(output.get("download_count", 0))
            total_error = int(output.get("error_count", 0))
        else:
            regex_download_count = "regex:.*SPMActiveIdle: {0}.*".format(url)
            regex_download_ok = "regex:.*SPMActiveIdle: Download complete.*"
            total_dl_msg = self._device_logger.get_message_triggered_status(regex_download_count)
            if total_dl_msg is None:
                total_download = 0
            else:
                total_download = len(total_dl_msg)
            self._device_logger.remove_trigger_message(regex_download_count)

            total_dl_ok = self._device_logger.get_message_triggered_status(regex_download_ok)
            if total_dl_ok is None:
                total_success = 0
            else:
                total_success = len(total_dl_ok)
            self._device_logger.remove_trigger_message(regex_download_ok)
            total_error = total_download - total_success
            output_msg = self._exec("adb shell am force-stop com.intel.spm.SPMActiveIdle")
            status = "SUCCESS"
        return status, total_download, total_error

    @need('wifi')
    def get_wifi_bandwidth(self, interface="wlan0"):
        """
        Get the Wifi current bandwidth

        :type interface: str
        :param interface: interface to check the bandwidth

        :rtype: String
        :return: the current bandwidth ("20", "40" or "80") in MHz
        """
        wifi_chipset = self._get_wifi_chipset_manufacturer()

        if wifi_chipset == self.CHIPSET_BROADCOM:

            cmd = "adb shell /system/bin/wlx -i %s status" % interface
            output = self._exec(cmd)

            if "/system/bin/wlx: not found" in output:
                msg = "/system/bin/wlx broadcom binary tool is not installed."
                msg += " Regulatory domain check is disabled."
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if "driver adapter not found" in output:
                msg = "Driver adapter not found - WiFi is probably OFF - WiFi must be ON to get the bandwidth"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            bandwidth = re.findall(r"Chanspec: [0-9.]*GHz channel [0-9]*[ ]([0-9]*)MHz", output)
            if bandwidth[0] not in ["20", "40", "80"]:
                msg = "Bandwidth value unknown : %s" % bandwidth
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            return bandwidth[0]

        elif wifi_chipset == self.CHIPSET_INTEL:
            cmd = "adb shell iw %s info" % interface
            output = self._exec(cmd)
            self._logger.debug(output)

            bandwidth = re.findall(r"width: ([0248]*) MHz", output)
            if bandwidth[0] not in ["20", "40", "80"]:
                msg = "Bandwidth value unknown : %s" % bandwidth
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            return bandwidth[0]

        elif wifi_chipset == self.CHIPSET_TI:
            msg = "Get bandwidth is not implement for this Wifi Chipset : %s" % str(wifi_chipset)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        else:
            msg = "Unknown Wifi Chipset : %s" % str(wifi_chipset)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    @need('wifi')
    def get_wifi_channel(self, interface="wlan0"):
        """
        Get the Wifi current channel

        :type interface: str
        :param interface: interface to check the channel

        :rtype: String
        :return: the current channel
        """
        """
        /!\ WARNING /!\
        This function is used to get the WiFi channel for STA and P2P mode. You just need to provide the good interface.
        Implementation is complete only for Broadcom chipsets. It uses linux commands to get the channel.

        A better implementation, which use the Android API, is provided by the function "get_wifi_direct_channel_freq" in
        WiFiDirect module. But it only works for WiFi Direct channels.
        """
        wifi_chipset = self._get_wifi_chipset_manufacturer()

        if wifi_chipset == self.CHIPSET_BROADCOM:

            cmd = "adb shell wlx -i %s status" % interface
            output = self._exec(cmd)

            channel = re.findall(r"Primary channel: ([0-9]*)", output)
            if channel is None:
                msg = "WiFi Channel can't be find : %s" % output
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if len(channel) != 1:
                msg = "Multiple channels found for WiFi : %s" % str(channel)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            return channel[0]

        elif wifi_chipset == self.CHIPSET_INTEL:
            cmd = "adb shell iw %s info" % interface
            output = self._exec(cmd)

            channel = re.findall(r"channel ([0-9]+)", output)
            if channel is None:
                msg = "WiFi Channel can't be find : %s" % output
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if len(channel) != 1:
                msg = "Multiple channels found for WiFi : %s" % str(channel)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            return channel[0]

        elif wifi_chipset == self.CHIPSET_TI:

            msg = "Get channel is not implement for this WiFi Chipset : %s" % str(wifi_chipset)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        else:
            msg = "Unknown WiFi Chipset : %s" % str(wifi_chipset)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    @need('wifi', False)
    def remove_wpa_certificates(self, credential_password=None, pin_code_removal=False):
        """
        Remove wpa certificates already installed on device.
        Possibility to remove pin code already set.

        :type pin_code_removal: boolean
        :param pin_code_removal: Remove Pin Code

        :return: None
        """
        KEYCODE_DPAD_DOWN = "20"
        KEYCODE_DPAD_CENTER = "23"
        KEYCODE_HOME = "3"

        display = self._device.get_uecmd("Display")

        # Unlock the device
        self._phone_system.set_phone_lock(0)

        # Force the screen orientation to portrait
        display.set_display_orientation("portrait")

        # Open Location and Security Settings directly
        self._exec(
                "adb shell am start -n com.android.settings/.SecuritySettings")

        # Erase all credential items and password
        count = 0
        while count < 15:
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
            count += 1
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        if pin_code_removal:
            # Remove Pin CODE
            msg = "Removing PIN CODE not defined for this android version"
            self._logger.info(msg)

        # Go back to home screen
        self._exec("adb shell input keyevent " + KEYCODE_HOME)
        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        display.set_display_orientation("auto")

    @need('modem')
    def get_available_networks(self):
        """
        Returns the list of all available networks.

        :rtype: list
        :return: a list of [available networks]
        """

        method = "getAvailableNetworks"

        msg = "Calling the method and waiting for 120 sec..."
        self._logger.info(msg)

        cell_descriptions = self._internal_exec_v2(
                self.network_type_module,
                method, timeout=120, is_system=True)

        return cell_descriptions
