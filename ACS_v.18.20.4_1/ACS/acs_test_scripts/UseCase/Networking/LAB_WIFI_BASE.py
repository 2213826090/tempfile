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
:summary: This file implements the LAB WIFI Base UC
:since: 22/08/2011
:author: szhen11
"""
import time
import os
import re
from datetime import datetime

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool_ex, TestConst
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiKeyExchangeTypes
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException


class LabWifiBase(UseCaseBase):

    """
    Lab Wifi Base Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize variables
        self._init_flight_mode = None
        self._connect_to_ap1 = False

        # Get FTP server parameters
        self._wifi_server = global_config.benchConfig.get_parameters("WIFI_SERVER")
        self._wifi_server_ip_address = self._wifi_server.get_param_value("IP")
        self._ftp_ip_address = self._wifi_server_ip_address
        self._ftp_username = self._wifi_server.get_param_value("username")
        self._ftp_password = self._wifi_server.get_param_value("password")
        if self._wifi_server.has_parameter("ftp_path"):
            self._ftp_path = self._wifi_server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Get Configurable AP server parameters
        default_ap = "CONFIGURABLE_AP1"
        self._current_ap = str(self._tc_parameters.get_param_value(
                               "CONFIGURABLE_AP_TO_USE",
                               default_value=default_ap))
        self._configurable_default_ap = None
        if self._current_ap == "":
            # Case of empty value
            self._current_ap = default_ap

        self._logger.info('Using access point "%s"' % self._current_ap)
        self._configurable_ap = global_config.benchConfig.\
            get_parameters(self._current_ap)
        if self._current_ap != default_ap:
            self._configurable_default_ap = global_config.benchConfig.\
                get_parameters(default_ap)

        self._ssid = self.__get_bc_ap_param("SSID")
        if self.__get_bc_ap_param("Model") == "AP_CONTROLLER" \
                and self._configurable_ap.has_parameter("APC_IP"):
            self._wifirouter_ip = \
                self._configurable_ap.get_param_value("APC_IP")
        else:
            self._wifirouter_ip = self._configurable_ap.get_param_value("IP")
        # Get EAP username from the test case.
        # If not available, take the bench config one.
        self._eap_user = self._tc_parameters.get_param_value("EAP_USER",
                                                             self.__get_bc_ap_param("EAP_user"))
        self._eap_password = self.__get_bc_ap_param("EAP_password")
        self._certificat_name = self.__get_bc_ap_param("CA_guard_password")

        eap_sim_user = self.__get_bc_ap_param("EAP_SIM_user")
        eap_aka_user = self.__get_bc_ap_param("EAP_AKA_user")

        self._mandatory_cert = str_to_bool_ex(self._tc_parameters.get_param_value("MANDATORY_CERT", "False"))
        if self._mandatory_cert is None:
            self._mandatory_cert = False

        # Name of the loaded certificate
        self._wpa_certificate_file = self.__get_bc_ap_param("certificate_file")
        self._passphrase_wep64 = self.__get_bc_ap_param("passphrase_WEP64")
        self._passphrase_wep128 = self.__get_bc_ap_param("passphrase_WEP128")
        self._passphrase_wpa = self.__get_bc_ap_param("passphrase_WPA")
        self._passphrase_wpa2 = self.__get_bc_ap_param("passphrase_WPA2")
        self._radiusip = self.__get_bc_ap_param("radiusip")
        self._radiusport = self.__get_bc_ap_param("radiusport")
        self._radiussecret = self.__get_bc_ap_param("radiussecret")

        # Get WIFI Router configuration according WIFI_HIDDEN
        self._hidden = str_to_bool_ex(self._tc_parameters.get_param_value("WIFI_HIDDEN", "False"))
        if self._hidden is None:
            self._logger.warning("UC parameter 'hidden' ignored: %s." % (str(self._hidden)))
            self._hidden = False

        # Get WIFI Router configuration according WIFI_STANDARD
        # ("a", "b", "g", "bg, "gn", "bgn", "n2.4G", "an" or "n5G")
        self._standard = self._tc_parameters.get_param_value("WIFI_STANDARD")

        # Get WIFI Router configuration according WIFI SECURITY
        self._security = self._tc_parameters.get_param_value("WIFI_SECURITY")

        # Get WIFI Router configuration according WIFI CHANNEL
        self._channel = self._tc_parameters.get_param_value("WIFI_CHANNEL")
        if type(self._channel) not in [type(''), type(u'')] or not self._channel.isdigit():
            self._logger.warning("UC parameter 'channel' ignored: %s." % (str(self._channel)))
            self._channel = None
            self._auto_channel = True
        else:
            self._auto_channel = False

        # Get WIFI Router configuration according WIFI DTIM
        self._dtim = self._tc_parameters.get_param_value("WIFI_DTIM")
        if type(self._dtim) not in [type(''), type(u'')] \
                or not self._dtim.isdigit():
            self._logger.warning("UC parameter 'dtim' ignored: %s."
                                 % (str(self._dtim)))
            self._dtim = None

        # Get WIFI Router configuration according WIFI BEACON
        self._beacon = self._tc_parameters.get_param_value("WIFI_BEACON")
        if type(self._beacon) not in [type(''), type(u'')] \
                or not self._beacon.isdigit():
            self._logger.warning("UC parameter 'beacon' ignored: %s."
                                 % (str(self._beacon)))
            self._beacon = None

        # Get WIFI Router configuration according WIFI WMM
        self._wmm = self._tc_parameters.get_param_value("WIFI_WMM")
        if type(self._wmm) not in [type(''), type(u'')] \
                or not self._wmm.isdigit():
            self._logger.warning("UC parameter 'wmm' ignored: %s."
                                 % (str(self._wmm)))
            self._wmm = None

        # Get WIFI Router configuration according WIFI BAND
        self._bandwidth = self._tc_parameters.get_param_value("WIFI_BAND")
        if self._bandwidth in [None, "", "None"]:
            self._bandwidth = "20"
            self._logger.warning("UC parameter 'bandwidth' not set: use default value 20MHz.")

        # Get the WIFI MIMO activation toggle
        self._wifi_mimo = str_to_bool_ex(self._tc_parameters.get_param_value("WIFI_MIMO", "False"))
        if self._wifi_mimo is None:
            self._wifi_mimo = False

        # Get the optional DUT wrong passphrase to test connection failure
        self._wrong_passphrase = str(self._tc_parameters.get_param_value("WIFI_PASSPHRASE"))

        self._eap_method = None
        self._phase2_auth = None
        self._passphrase = None
        if self._security in ["WEP64", "WEP64-OPEN"]:
            self._passphrase = self._passphrase_wep64
        elif self._security in ["WEP128", "WEP128-OPEN"]:
            self._passphrase = self._passphrase_wep128
        elif self._security in ["EAP-WPA", "EAP-WPA2"]:

            # Get WIFI Router configuration according to WPA-enterprise options
            self._eap_method = str(self._tc_parameters.get_param_value("EAP_METHOD"))
            self._phase2_auth = str(self._tc_parameters.get_param_value("PHASE2_AUTH"))

            if self._eap_method == "SIM":
                self._eap_user = eap_sim_user
                # If no username is required, we don't need the password
                if eap_sim_user == "":
                    self._eap_password = ""
            elif self._eap_method == "AKA":
                self._eap_user = eap_aka_user
                # If no username is required, we don't need the password
                if eap_aka_user == "":
                    self._eap_password = ""
            # Compute passphrase for WPA-ENT (will be parsed by ACS Embedded)
            self._passphrase = self._eap_method + "-" + self._phase2_auth \
                + "_" + self._eap_user + "_" + self._eap_password \
                + "_" + self._certificat_name + "_" + str(self._mandatory_cert)

            if self._wrong_passphrase not in ("None", "NONE", "none"):
                self._wrong_passphrase = self._eap_method + "-" \
                    + self._phase2_auth + "_" + self._eap_user + "_" \
                    + self._wrong_passphrase + "_" + self._certificat_name

        elif "WPA2" in self._security:
            self._passphrase = self._passphrase_wpa2
        elif "WPA" in self._security:
            self._passphrase = self._passphrase_wpa

        # Key exchange parameters
        self._key_exchange_mode = str(self._tc_parameters.get_param_value("KEY_EXCHANGE_MODE"))

        if self._key_exchange_mode.lower() not in ["none", ""]:
            # Get optional key exchange parameters
            self._key_exchange_pin = str(self._tc_parameters.get_param_value("KEY_EXCHANGE_PIN"))
            if self._key_exchange_pin.lower() in ["none", ""]:
                self._key_exchange_pin = None
            self._key_exchange_should_fail = str_to_bool_ex(self._tc_parameters.get_param_value("MAKE_WPS_TEST_FAIL",
                                                                                                "False"))
            if self._key_exchange_should_fail is None:
                self._key_exchange_should_fail = False
        else:
            self._key_exchange_mode = None
            self._key_exchange_should_fail = False

        # Disable the DUT wrong_passphrase if equals to the passphrase \
        # to set to the AP or if in OPEN mode
        if self._security == "OPEN" \
                or self._passphrase == self._wrong_passphrase:
            self._wrong_passphrase = 'None'

        # Get DUT static ip configuration according IP_SETTING
        self._ip_setting_enable = str_to_bool_ex(self._tc_parameters.get_param_value("IP_SETTING_ENABLE", "False"))
        if self._ip_setting_enable is None:
            self._ip_setting_enable = False
        self._ip_address = str(self._tc_parameters.get_param_value("IP_ADDRESS"))
        self._netmask = str(self._tc_parameters.get_param_value("NETMASK"))
        self._gateway = str(self._tc_parameters.get_param_value("GATEWAY"))
        self._dns1 = str(self._tc_parameters.get_param_value("DNS1"))
        self._dns2 = str(self._tc_parameters.get_param_value("DNS2"))
        # DHCP or STATIC ip settings
        if self._ip_setting_enable:
            self._ip_setting = "static"
        else:
            self._ip_setting = "dhcp"

        # initial wifi equipment
        self._ns = self._em.get_configurable_ap(self._current_ap)

        # Load sniffer equipment if requested
        self._sniff_requested = False
        self._sniffer = None
        self._capture = None
        sniff = str(self._tc_parameters.get_param_value("WIFI_SNIFF"))
        if sniff.lower() in ("1", "on"):
            self._sniff_requested = True
        else:
            self._sniff_requested = False
        if self._sniff_requested or self._bandwidth == "40" or self._wifi_mimo:
            self._sniffer = self._em.get_sniffer("WIFI_SNIFFER1")

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")
        self._appmgmt_api = self._device.get_uecmd("AppMgmt")
        # DHCP state unknown at beginning.
        self._dut_dhcp_state = "ERROR"

        # Get the regulatory domain to use for the test
        self._user_reg_domain = self._tc_parameters.get_param_value("REGULATORY_DOMAIN")

        self._dut_auto_reg_domain = str(self._dut_config.get("autoWifiRegulatoryDomain")).lower() == "true"

        # Should we set flight mode?
        flight_mode = str_to_bool_ex(self._tc_parameters.get_param_value("FLIGHT_MODE", "False"))
        if flight_mode:
            self._use_flight_mode = 1
        else:
            self._use_flight_mode = 0

        # Packages to be disabled
        self._disabled_packages_list = \
            str(self._tc_parameters.get_param_value("PACKAGES_TO_DISABLE"))
        if self._disabled_packages_list.lower() not in ["", "none"]:
            self._disabled_packages_list = self._disabled_packages_list.split(',')
        else:
            self._disabled_packages_list = None
        # Used to store packages initial state.
        self._packages_initial_state = []

        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._sleep_mode_api = self._device.get_uecmd("SleepMode")

        # Set BT prerequisite (set ON, set OFF or nothing) for FIT BT/WIFI tests
        self._bt_fit_used, self._bt_wished_value = self._get_bt_fit_config()
        # BT initial state
        self._bt_initial_state = 'STATE_ON'

        # Get IPV6 configuration
        self._use_ipv6 = str_to_bool_ex(self._tc_parameters.get_param_value("USE_IPV6", "False"))
        if self._use_ipv6 is None:
            self._use_ipv6 = False
        if self._use_ipv6:
            self._ipv6_configuration_mode = str(self._tc_parameters.get_param_value("IPV6_CONFIGURATION_MODE"))
            self._computer_name = str(self._tc_parameters.get_param_value("IPV6_COMPUTER", "COMPUTER2"))
            self._logger.info('Using computer "%s"' % self._computer_name)
            self._computer = None
            self._ipv6prefix = ""
            # IPV6 need the screen to be powered on to receive router a advertisement packets.
            self._current_screen_timeout = 0
            self._wlan_phy_device = str(self._dut_config.get("wlanPhyDevice"))

        self._dut_wlan_iface = str(self._dut_config.get("wlanInterface"))
        self._connection_time = -1

        # Pointer to the latest character read in the sniff file to check specific connection features
        self._sniff_pointer = 0

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)
        return self.__set_up(True)

    def set_up_without_connect(self):
        """
        LAB_WIFI_BASE set_up without connecting DUT to AP
        """
        UseCaseBase.set_up(self)
        return self.__set_up(False)

    def set_up_without_flightmode(self, networking_api=None):
        """
        LAB_WIFI_BASE set_up without connecting DUT to AP

        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        UseCaseBase.set_up(self)
        return self.__set_up(True, False, networking_api)

    def __set_up(self, connection_requested=True, flightmode_mgnt=True, networking_api=None):
        """
        Private setup used by public set_up

        :type flightmode_mgnt: Boolean
        :param flightmode_mgnt: Handle the management of the status of Flight mode
        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        # Set the networking API to its default value if not given as parameter
        self._connect_to_ap1 = connection_requested
        if networking_api is None:
            networking_api = self._networking_api

        # Configure IP v6 if required
        self._configure_ipv6(networking_api)

        # DUT initial state
        self._set_dut_initial_state(flightmode_mgnt, networking_api)

        # Configure the AP
        self._configure_ap(networking_api)

        # Configure the DUT after Wifi has been powered ON (in _configure_ap)
        networking_api.set_wifi_frequency_band("auto", True, self._dut_wlan_iface)
        time.sleep(self._wait_btwn_cmd)
        networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)
        networking_api.request_wifi_scan()
        time.sleep(self._wait_btwn_cmd)

        # Set DUT Wifi Configuration
        self._set_dut_wifi_configuration(networking_api)

        if self._sniffer:
            # Initialize Sniffer object
            self._sniffer.init()

            # Start the sniffer if Sniff is requested for the whole test
            if self._sniff_requested:
                self._start_sniffer(self._channel, self._ssid,
                                    self._bandwidth == "40" or self._wifi_mimo, networking_api)

        # Connect the DUT to the Wifi Access Point
        if connection_requested:
            self._wifi_connect_dut(networking_api)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)
        return self.__tear_down()

    def tear_down_without_flightmode(self, networking_api=None):
        """
        End and dispose the test without handling flightmode

        :type flightmode_mgnt: Boolean
        :param flightmode_mgnt: Handle the management of the status of Flight mode
        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        UseCaseBase.tear_down(self)
        return self.__tear_down(False, networking_api)

    def __tear_down(self, flightmode_mgnt=True, networking_api=None):
        """
        Main tear_down private function

        :type flightmode_mgnt: Boolean
        :param flightmode_mgnt: Handle the management of the status of Flight mode
        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        time.sleep(self._wait_btwn_cmd)

        # Set the networking API to its default value if not given as parameter
        if networking_api is None:
            networking_api = self._networking_api

        # stop sniffing
        if self._sniffer:
            if self._sniff_requested and self._capture:
                self._sniffer.stop_capture(self.test_ok)
            self._sniffer.release()

        # Cleanup DUT known WIFI configuration
        networking_api.wifi_remove_config(self._ssid)
        time.sleep(self._wait_btwn_cmd)

        # Restore DUT state (flightmode, BT, disabled packages)
        self._restore_dut_state(flightmode_mgnt, networking_api)

        # Disable radio for all Wifi APs
        self._disable_all_radios()

        # Cleanup IP v6 settings
        self._reset_ipv6(networking_api)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _set_dut_initial_state(self, flightmode_mgnt, networking_api):
        """
        Set the initial state for the DUT:
        - Flightmode status
        - BT feature status
        - Packages list to disable

        :type flightmode_mgnt: Boolean
        :param flightmode_mgnt: Handle the management of the status of Flight mode
        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        # Instantiate generic UECmd
        localconnectivity_api = self._device.get_uecmd("LocalConnectivity", True)
        # Package deactivation
        if self._disabled_packages_list is not None:
            self._disable_packages(self._disabled_packages_list)

        # Handle Flightmode status and BT status during this Wifi test
        if flightmode_mgnt:
            # Initial state for flightmode:
            self._init_flight_mode = networking_api.get_flight_mode()
        if self._bt_fit_used:
            # Initial state for BT:
            self._bt_initial_state = localconnectivity_api.get_bt_power_status()

        # Handle the Flight mode status during the test
        if flightmode_mgnt and self._use_flight_mode != self._init_flight_mode:
            networking_api.set_flight_mode(self._use_flight_mode)

            # Set desired BT status during this test (BT status should have been changed by flightmode)
            if self._bt_fit_used:
                localconnectivity_api.set_bt_power(self._bt_wished_value)
                time.sleep(self._wait_btwn_cmd)
        else:
            # If BT is not at the wished value, set it to the correct one.
            if self._bt_fit_used and self._bt_initial_state != self._bt_wished_value:
                localconnectivity_api.set_bt_power(self._bt_wished_value)
                time.sleep(self._wait_btwn_cmd)

#------------------------------------------------------------------------------

    def _restore_dut_state(self, flightmode_mgnt, networking_api):
        """
        Restore the initial state for the DUT, as found when entering in set_up
        - Flightmode status
        - BT feature status
        - Packages list to re-enable

        :type flightmode_mgnt: Boolean
        :param flightmode_mgnt: Handle the management of the status of Flight mode
        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        # Instantiate generic UECmd
        localconnectivity_api = self._device.get_uecmd("LocalConnectivity", True)
        # Restore flightmode status
        if flightmode_mgnt and self._init_flight_mode is not None and self._use_flight_mode != self._init_flight_mode:
            networking_api.set_flight_mode(self._use_flight_mode)

        # Restore Bt in initial state
        if self._bt_fit_used:
            localconnectivity_api.set_bt_power(self._bt_initial_state)
            time.sleep(self._wait_btwn_cmd)

        # Turn WIFI off
        networking_api.set_wifi_power("off")

        # Restore package status list
        if self._disabled_packages_list is not None and self._packages_initial_state != []:
            for cur_pack in self._packages_initial_state:
                if len(cur_pack) >= 2:
                    if cur_pack[1] == "enabled":
                        state = True
                    else:
                        state = False
                    self._appmgmt_api.app_enable_disable(cur_pack[0], state)
                else:
                    msg = "Can't Restore package %s original state" % cur_pack[0]
                    self._logger.warning(msg)

#------------------------------------------------------------------------------

    def _configure_ap(self, networking_api):
        """
        Sub function to configure the AP

        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        # Initiate connection to the equipment
        self._ns.init()

        try:
            # Regulatory Domain management
            self._set_regulatory_domain('AP', networking_api)

            # Configure the equipment
            self._ns.set_wifi_config(self._ssid,
                                     self._hidden,
                                     self._standard,
                                     self._security,
                                     self._passphrase,
                                     self._channel,
                                     self._dtim,
                                     self._beacon,
                                     self._wmm,
                                     self._bandwidth,
                                     self._wifi_mimo,
                                     self._radiusip,
                                     self._radiusport,
                                     self._radiussecret)

            self._ns.enable_wireless()

            # Turn Wifi interface ON
            networking_api.set_wifi_power("on")

            if self._wait_btwn_cmd <= 2:
                time.sleep(2)
            else:
                time.sleep(self._wait_btwn_cmd)

            # Regulatory Domain management
            self._set_regulatory_domain('DUT', networking_api)
            time.sleep(self._wait_btwn_cmd)

            # Do the key exchange if needed.
            if self._key_exchange_mode is not None:
                if(self._key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_AP) and\
                        (self._key_exchange_pin is None):
                    # Get pin code from AP.
                    self._key_exchange_pin = self._ns.get_wifi_wps_pin()

            if self._sniffer and self._auto_channel:
                # If using auto channel, retrieve the selected channel from the AP
                self._channel = self._ns.get_selected_channel()

        finally:
            # Close the connection to AP
            self._ns.release()

#------------------------------------------------------------------------------

    def _set_dut_wifi_configuration(self, networking_api):
        """
        Set the Wifi Configuration to the DUT

        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """

        if self._key_exchange_mode is None:
            if str(self._wrong_passphrase).lower() == 'none':
                # If we use a TLS enterprise security, we need to enter the pin code to ensure
                # the certificate is unlocked and the supplicant have access to its private key.
                if self._eap_method == "TLS" or self._mandatory_cert:
                    phone_pin_lock = str(self._configurable_ap.get_param_value("Credential_password", "12345678"))
                    networking_api.simulate_phone_pin_unlock(phone_pin_lock)
                networking_api.set_wificonfiguration(self._ssid,
                                                     self._passphrase,
                                                     self._security,
                                                     self._ip_setting,
                                                     self._ip_address,
                                                     self._netmask,
                                                     self._gateway,
                                                     self._dns1,
                                                     self._dns2)
            else:
                networking_api.set_wificonfiguration(self._ssid,
                                                     self._wrong_passphrase,
                                                     self._security,
                                                     self._ip_setting,
                                                     self._ip_address,
                                                     self._netmask,
                                                     self._gateway,
                                                     self._dns1,
                                                     self._dns2)

#------------------------------------------------------------------------------

    def _wifi_connect_dut(self, networking_api=None):
        """
        Launch the Connection from the DUT to the Wifi Access Point

        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        # Set the networking API to its default value if not given as parameter
        if networking_api is None:
            networking_api = self._networking_api

        # start sniffing if not already started in __set_up
        self._init_connection_analysis(networking_api=networking_api)

        if self._key_exchange_mode is None:
            if str(self._wrong_passphrase).lower() == 'none':
                # monitor wifi connection time
                start_time = time.time()
                # Connect the DUT on the Wifi network
                networking_api.wifi_connect(self._ssid)
                end_time = time.time()
                time.sleep(self._wait_btwn_cmd)

                # Control 40MHz and/or MIMO connection if enabled
                self._run_connection_analysis_process()

                # IPV6 specific parameters / DUT part
                if self._use_ipv6 and self._ip_setting == "dhcp":
                    if "rdnssd" in self._ipv6_configuration_mode.lower():
                        networking_api.launch_rdnssd_client(self._dut_wlan_iface)
                    else:
                        # Dynamic Host Configuration Protocol mode
                        networking_api.launch_dhcp6_client(self._ipv6_configuration_mode, self._dut_wlan_iface)
                    time.sleep(self._wait_btwn_cmd)

                # Now we've got to ensure that the connection at IP level is actually done.
                # If we ask for the DUT's IP address with get_wifi_ip_address just after
                # leaving this function, the DUT may return either the last good IP address or
                # 0.0.0.0. To avoid this, we ensure that the connection is not only
                # established with the AP (AUTHENTICATED state in android) but the IP
                # connection is also there (CONNECTED state).
                if not self._ip_setting_enable:
                    # Dynamic IP setting. We don't need this for static IP address.
                    networking_api.wait_for_wifi_dhcp_connected()
                    time.sleep(self._wait_btwn_cmd)

                if self._use_ipv6:
                    time.sleep(5)
                    networking_api.check_ipv6_consistency(self._dut_wlan_iface, self._ipv6prefix)
                    time.sleep(self._wait_btwn_cmd)
            else:
                # Connection Failure case
                # Try to connect the DUT on the Wifi network (with wrong passphrase, it should failed)
                start_time = time.time()
                networking_api.wifi_connect(self._ssid, False)
                end_time = time.time()

            # Compute connection time
            self._connection_time = end_time - start_time

        else:
            # Key exchange authentication (WPS for instance)
            self._connection_time = self._wifi_wps_connect_dut(networking_api)

#------------------------------------------------------------------------------

    def _wifi_wps_connect_dut(self, networking_api):
        """
        Launch the Connection from the DUT to the Wifi Access Point, using WPS

        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        :rtype: float
        :return: the connection time
        """
        # Set equipment part
        self._ns.init()
        try:
            self._ns.set_wifi_key_exchange(self._key_exchange_mode, self._key_exchange_pin)
        finally:
            self._ns.release()

        start_time = time.time()
        # In order to set -1 as _connection_time, if we are in WPS and the connection should fail,
        # we init end_time like this:
        end_time = start_time - 1

        # Set DUT part
        networking_api.wifi_setkeyexchange(self._ssid,
                                           self._key_exchange_mode,
                                           self._key_exchange_pin,
                                           self._key_exchange_should_fail,
                                           self._dut_wlan_iface)

        if not self._key_exchange_should_fail:
            if not self._ip_setting_enable:
                # Dynamic IP setting. We don't need this for static IP address.
                networking_api.wait_for_wifi_dhcp_connected()
            networking_api.check_connection_state(self._ssid)
            end_time = time.time()

            # Control 40MHz and/or MIMO connection if enabled
            self._run_connection_analysis_process()

        else:
            # Connection Failure case
            state = networking_api.get_wifi_dhcp_state()
            if state != "DISCONNECTED":
                msg = "Connection shouldn't be established!"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return end_time - start_time

    def _configure_ipv6(self, networking_api):
        """
        Configure IP v6 settings if IPv6 is enabled for the test

        :type networking_api: Networking API object
        :param networking_api: networking API object of the DUT to control
        """
        if self._use_ipv6:
            # IPV6 specific parameters / Host part
            self._computer = self._em.get_computer(self._computer_name)
            self._computer.start_stop_service("radvd", "start")
            self._ipv6prefix = str(self._computer.get_ipv6_prefix())

            if self._ip_setting == "dhcp":
                if "rdnssd" in self._ipv6_configuration_mode.lower():
                    # Recursive DNS Server mode
                    self._computer.start_stop_service("wide-dhcpv6-server", "stop")
                else:
                    # Dynamic Host Configuration Protocol mode
                    self._computer.start_stop_service("wide-dhcpv6-server", "start")
            else:
                self._computer.start_stop_service("wide-dhcpv6-server", "stop")

            # IPV6 need the screen to be powered on to receive router a advertisement packets.
            self._current_screen_timeout = self._phone_system_api.get_screen_timeout()
            self._phone_system_api.set_screen_timeout(240)
            self._phone_system_api.display_on()

            # In order to receive router advertising packets, we need to disable IP filters
            networking_api.disable_ip_filtering(self._wlan_phy_device)

#------------------------------------------------------------------------------

    def _reset_ipv6(self, networking_api):
        """
        Clean up the IP v6 configuration
        """
        if self._use_ipv6:
            self._computer.start_stop_service("radvd", "stop")
            if not self._ip_setting_enable:
                self._computer.start_stop_service("wide-dhcpv6-server", "stop")

            # Set the screen timeout to default
            self._phone_system_api.set_screen_timeout(self._current_screen_timeout)
            self._phone_system_api.display_off()
            networking_api.enable_ip_filtering(self._wlan_phy_device)

#------------------------------------------------------------------------------

    def _set_regulatory_domain(self, device, networking_api=None):
        """
        Sub-function to set regulatory domain to whom it is required (DUT / AP)

        WARNING: Wifi interface on DUT should not been turned OFF/ON after
        setting regulatory domain. Otherwise default value should be restored.

        This function does nothing if both the DUT and the AP are regulatory domain auto configurable.

        :type device: str
        :param device: 'AP' or 'DUT'
        :type networking_api: Networking object
        :param networking_api: UECmd Networking API
        """
        # Set the networking API to its default value if not given as parameter
        if networking_api is None:
            networking_api = self._networking_api

        # Do nothing if auto reg domain is enabled
        if self._dut_auto_reg_domain and self._ns.is_regulatorydomain_sent():
            return

        if self._ns.is_regulatorydomain_configurable() and self._user_reg_domain:
            # In this case, Regulatory domain is set using user defined value in TC parameters
            if device == 'DUT':
                networking_api.set_regulatorydomain(self._user_reg_domain, self._dut_wlan_iface)
            elif device == 'AP':
                self._ns.set_regulatorydomain(self._user_reg_domain)

        elif device == 'DUT':
            # In this case, Regulatory domain is set using fixed value read from AP
            reg_domain = self._ns.get_regulatorydomain()
            if reg_domain:
                networking_api.set_regulatorydomain(reg_domain, self._dut_wlan_iface)
            else:
                self._logger.warning("Unable to set regulatory domain for the DUT. Using the DUT default one")

#------------------------------------------------------------------------------

    def _disable_packages(self, packages_list):
        """
        Packages deactivation

        :type packages_list: List
        :param packages_list: list of packages to disable on the DUT
        """
        # Package deactivation
        for cur_pack in packages_list:
            cur_pack = cur_pack.strip()
            # Save original state to restore it at teardown.
            cur_status = self._appmgmt_api.get_app_status(cur_pack)
            # If package is already disable, don't change its state.
            if cur_status == TestConst.STR_ENABLE:
                self._packages_initial_state.append((cur_pack, cur_status))
                # Disable package
                self._appmgmt_api.app_enable_disable(cur_pack, False)

#------------------------------------------------------------------------------

    def _disable_all_radios(self):
        """
        Disable radios of all Controllable APs
        """
        self._logger.info("Disable all radios")

        aps = self._em.get_all_configurable_aps()
        for ap_api in aps:
            if not ap_api.is_radio_off():
                ap_api.init()
                ap_api.disable_wireless()
                ap_api.release()
                time.sleep(self._wait_btwn_cmd)

#------------------------------------------------------------------------------

    def __get_bc_ap_param(self, param):
        """
        Get the AP parameter value from the bench config.
        In case of not using the default AP, and absence of parameter,
        it returns the value for the default AP.

        :type param: str
        :param param: Bench config AP parameter name
        :rtype: String
        :return: Value of the parameter in the bench configuration file
        """
        if not self._configurable_ap:
            # This error should not occurs in production
            msg = "UseCase coding error. " + \
                "Please instantiate the AP BenchConfig parameters object"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INSTANTIATION_ERROR, msg)

        # Retrieve the default value if exists
        default_value = None
        if self._configurable_default_ap \
                and self._configurable_default_ap.has_parameter(param):
            default_value = self._configurable_default_ap.\
                get_param_value(param)

        # Retrieve the bench config value
        value = self._configurable_ap.get_param_value(param, default_value)

        return value

#------------------------------------------------------------------------------

    def _get_bt_fit_config(self):
        """
        Get BT configuration for FIT BT/WIFI tests

        :rtype: list of 2 elements (boolean, str)
        :return: true if BT/WIFI FIT is used, Bluetooth state ON or OFF for the test to be run
        """
        # Read WHILE_BLUETOOTH_ON parameter (named for retro-compatibility) from test case xml file for FIT tests
        param_while_bt_on = \
            str(self._tc_parameters.get_param_value("WHILE_BLUETOOTH_ON"))

        if param_while_bt_on.lower() in ["1", "on", "true", "yes"]:
            bt_fit_used = True
            bt_wished_value = 'STATE_ON'
        elif param_while_bt_on.lower() in ["0", "off", "false", "no"]:
            bt_fit_used = True
            bt_wished_value = 'STATE_OFF'
        else:
            bt_fit_used = False
            bt_wished_value = 'STATE_OFF'

        return bt_fit_used, bt_wished_value

#------------------------------------------------------------------------------

    def _run_connection_analysis_process(self, bandwidth=None, wifi_mimo=None):
        """
        Control specific features (40MHz / MIMO) are well activated during the WIFI connection negotiation
        For that, it is required to analyze the sniffer log.

        :type bandwidth: String
        :param bandwidth: force check bandwidth option "20" or "40"
        :type wifi_mimo: Boolean
        :param wifi_mimo: Force check MIMO option
        """
        # If no specific parameter has been passed to the function and XML TC parameter uses specific feature
        if bandwidth is None and self._bandwidth != "40" and wifi_mimo is None and not self._wifi_mimo:
            # No specific feature to test
            return

        # Use XML TC parameters only if both bandwidth and wifi_mimo function parameters are empty
        # and if TC parameters uses specific features
        if bandwidth is None and wifi_mimo is None:
            if self._bandwidth == "40":
                bandwidth = self._bandwidth
            if self._wifi_mimo:
                wifi_mimo = self._wifi_mimo

        if not self._capture:
            msg = "Wifi sniffer has not been started. We cannot check specific feature(s)"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        if self._sniff_requested:
            # Sniffer log must not be interrupted
            self._sniffer.get_capture_file(self._capture)
        else:
            # Sniffer is not requested anymore
            self._sniffer.stop_capture(False)

        try:
            # For 40MHz bandwidth, control that the connection has well been established in 40MHz
            if bandwidth == "40":
                self._check_connection_is_40mhz(self._capture)
            elif bandwidth == "20":
                self._check_connection_is_20mhz(self._capture)

            # For MIMO, control that the connection has well been established with MIMO feature enabled
            if wifi_mimo:
                self._check_connection_is_mimo(self._capture)
            elif wifi_mimo is False:
                self._check_connection_is_not_mimo(self._capture)
        finally:
            # Delete the temporary file
            if os.path.exists(self._capture):
                os.remove(self._capture)

#------------------------------------------------------------------------------

    def _check_connection_is_40mhz(self, sniffer_log_file):
        """
        Analyze if connection logs tells that
        - AP is able to accept 40MHz connection
        - DUT negotiated a 40MHz connection to the AP.

        :type sniffer_log_file: str
        :param sniffer_log_file: file path to the local sniffer log file to parse
        """
        self.__check_bandwidth_connection(sniffer_log_file, True)

    def _check_connection_is_20mhz(self, sniffer_log_file):
        """
        Analyze if connection logs tells that
        - AP is configure with 20MHz channel
        - DUT negotiated a 20MHz connection to the AP.

        :type sniffer_log_file: str
        :param sniffer_log_file: file path to the local sniffer log file to parse
        """
        self.__check_bandwidth_connection(sniffer_log_file, False)

    def __check_bandwidth_connection(self, sniffer_log_file, test_40mhz_enable):
        """
        Analyze connection logs (retrieve from sniffer) and inform about the connection bandwidth

        :type sniffer_log_file: str
        :param sniffer_log_file: file path to the local sniffer log file to parse
        :type test_40mhz_enable: boolean
        :param test_40mhz_enable: True to check a connection using 40MHz channel, False to test 20MHz channel
        """
        support_channel_width_mask = 0x2
        beacon_checked = False
        association_request_checked = False
        in_frame = False
        frame_is_beacon = False
        frame_is_association_request = False

        if test_40mhz_enable:
            expected_value = support_channel_width_mask
            msg_log = "40MHz enabled"
        else:
            expected_value = 0
            msg_log = "20MHz enabled"

        if not os.path.isfile(sniffer_log_file):
            msg = "Sniffer log file does not exist"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        # Open the sniffer log file to parse
        sniff_file = open(sniffer_log_file)
        # Set the read pointer at the beginning of the unparsed part.
        sniff_file.seek(self._sniff_pointer)
        self._logger.debug("SEEK %d in SNIFFER log file" % self._sniff_pointer)

        self.get_logger().debug("Start sniffer log bandwidth parsing")
        try:
            # We cannot use "for line in sniff_file:", because thus sniff_file.tell() will not be relevant
            nextline = "\n"
            # EOF is ""
            while nextline != "":

                # Read the next line
                nextline = sniff_file.readline()

                # Find the beginning of the the first frame; packet start with the key word 'Frame'
                if nextline.startswith('Frame'):
                    # New frame detected
                    in_frame = True
                    frame_is_beacon = False
                    frame_is_association_request = False

                if not in_frame:
                    continue

                if not beacon_checked:

                    if not frame_is_beacon:
                        if nextline.startswith('IEEE 802.11 Beacon frame'):
                            frame_is_beacon = True
                            self.get_logger().debug("Frame is beacon")
                        continue

                    if "HT Capabilities Info" in nextline:
                        cap_info = re.findall(r'HT Capabilities Info:[ ]*(0x[0-9A-Fa-f]+)', nextline)
                        if len(cap_info) > 0 and (int(cap_info[0], 16) & support_channel_width_mask) == expected_value:
                            # HT capability info of the beacon frame (sent by the AP) is valid
                            beacon_checked = True
                            self.get_logger().debug("--20/40MHz information found for AP: %s--" % msg_log)
                            # We can now ignore the end of the Frame
                            in_frame = False

                # if HT capability info is valid in the beacon, then check the assoc request HT info
                else:

                    if not frame_is_association_request:
                        if nextline.startswith('IEEE 802.11 Association Re'):
                            frame_is_association_request = True
                            self.get_logger().debug("Frame is association request or response")
                        continue

                    if "HT Capabilities Info" in nextline:
                        cap_info = re.findall(r'HT Capabilities Info:[ ]*(0x[0-9A-Fa-f]+)', nextline)
                        if len(cap_info) > 0 and (int(cap_info[0], 16) & support_channel_width_mask) == expected_value:
                            # HT capability info of the association frame (sent by the DUT) is valid
                            association_request_checked = True
                            self.get_logger().debug("--20/40MHz information found for DUT connection: %s--" % msg_log)
                            break

        finally:
            sniff_file.close()
            self.get_logger().debug("Stop sniffer log 40MHz parsing")

        if not association_request_checked or not beacon_checked:
            if test_40mhz_enable:
                msg = "DUT not connected AP with 40 MHz channel"
            else:
                msg = "20 MHz channel setup is not correct "
                msg += "(AP is configured with 40MHz or DUT believes he is in 40MHz)"
            self._logger.error(msg)

            # Cisco1250 and Cisco1260 AP does not broadcast 20MHz info in its beacon.
            # So the log analysis always leads to a false positive issue.
            if test_40mhz_enable or not self._ns.get_model().startswith("CISCO_12"):
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def _check_connection_is_mimo(self, sniffer_log_file):
        """
        Analyze if connection logs tells that
        - MIMO feature is enable on AP
        - DUT negotiated the connection to AP with MIMO.

        :type sniffer_log_file: str
        :param sniffer_log_file: file path to the local sniffer log file to parse
        """
        self.__check_mimo_connection(sniffer_log_file, "Rx Bitmask Bits 8-15: 0x000000ff")

    def _check_connection_is_not_mimo(self, sniffer_log_file):
        """
        Analyze if connection logs tells that
        - MIMO feature is not enable on AP
        - DUT negotiated the connection to AP without MIMO.

        :type sniffer_log_file: str
        :param sniffer_log_file: file path to the local sniffer log file to parse
        """
        self.__check_mimo_connection(sniffer_log_file, "Rx Bitmask Bits 8-15: 0x00000000")

    def __check_mimo_connection(self, sniffer_log_file, pattern):
        """
        Check if MIMO feature is activated for the last WIFI connection.

        :type sniffer_log_file: str
        :param sniffer_log_file: file path to the local sniffer log file to parse
        :type pattern: str
        :param pattern: Pattern to look for in the log in order to check MIMO feature
        """
        beacon_checked = False
        association_request_checked = False
        in_frame = False
        frame_is_beacon = False
        frame_is_association_request = False
        in_mcs_details = False

        # Rely on the last 2 digits to know if we will check if MIMO is enabled or disabled
        if pattern[-2:] == "00":
            status = "disabled"
        else:
            status = "enabled"

        if not os.path.isfile(sniffer_log_file):
            msg = "Sniffer log file does not exist"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        # Open the sniffer log file to parse
        sniff_file = open(sniffer_log_file)
        # Set the read pointer at the beginning of the unparsed part.
        sniff_file.seek(self._sniff_pointer)
        self._logger.debug("SEEK %d in SNIFFER log file" % self._sniff_pointer)

        self.get_logger().debug("Start sniffer log MIMO parsing")
        try:
            # We cannot use "for line in sniff_file:", because thus sniff_file.tell() will not be relevant
            nextline = "\n"
            # EOF is ""
            while nextline != "":

                # Read the next line
                nextline = sniff_file.readline()

                # Find the beginning of the the first frame; packet start with the key word 'Frame'
                if nextline.startswith('Frame'):
                    # Reset flags each time we enter a new Frame
                    in_frame = True
                    frame_is_beacon = False
                    frame_is_association_request = False
                    in_mcs_details = False

                if not in_frame:
                    continue

                if not beacon_checked:
                    # First we need to check if AP enables MIMO feature
                    if not frame_is_beacon:
                        if nextline.startswith('IEEE 802.11 Beacon frame'):
                            frame_is_beacon = True
                            self.get_logger().debug("Frame is beacon")
                        continue

                    # Find and analyze MCS lines
                    if "Rx Modulation and Coding Scheme (One bit per modulation)" in nextline:
                        in_mcs_details = True
                        continue

                    if in_mcs_details:
                        # Search in all following lines that look like a Bit Mask decomposition if MCS 8-15 are enabled
                        if "Rx Bitmask Bits" not in nextline:
                            in_mcs_details = False
                            continue

                        # Search for the value for 8-15 bitmask
                        if pattern in nextline:
                            # AP enables MIMO feature
                            beacon_checked = True
                            self.get_logger().debug("--Beacon checked: MIMO is %s on AP--" % status)
                            # We can now ignore the end of the Frame
                            in_frame = False

                else:
                    # Second, if MIMO info is valid in the beacon, then check the assoc request MIMO info
                    if not frame_is_association_request:
                        if nextline.startswith('IEEE 802.11 Association Re'):
                            frame_is_association_request = True
                            self.get_logger().debug("Frame is association request or response")
                        continue

                    # Find and analyze MCS lines
                    if "Rx Modulation and Coding Scheme (One bit per modulation)" in nextline:
                        in_mcs_details = True
                        continue

                    if in_mcs_details:
                        # Search in all following lines that look like a Bit Mask decomposition if MCS 8-15 are enabled
                        if "Rx Bitmask Bits" not in nextline:
                            in_mcs_details = False
                            continue

                        # Search for the value for 8-15 bitmask
                        if pattern in nextline:
                            # DUT connection enables MIMO
                            association_request_checked = True
                            self.get_logger().debug("--Association Request checked: MIMO connection is %s--" % status)
                            # We can now exit the file parsing process
                            break

        finally:
            sniff_file.close()
            self.get_logger().debug("Stop sniffer log MIMO parsing")

        if not association_request_checked or not beacon_checked:
            msg = "MIMO is not %s" % status
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

#------------------------------------------------------------------------------

    def _start_sniffer(self, channel, ssid, ascii_mode, networking_api=None):
        """
        build a unique capture file name with a time stamp inside the _Report directory
        :type channel: str
        :param channel: channel in which the sniffer must be started
        :type ssid: str
        :param ssid: ssid in which the sniff should be filtered
        :type ascii_mode: boolean
        :param ascii_mode: Save sniffer log in ascii mode (used to be able to parse the log automatically)
        :type networking_api: Networking object
        :param networking_api: UECmd Networking API
        """
        # Set the networking API to its default value if not given as parameter
        if networking_api is None:
            networking_api = self._networking_api

        filename = "capture-%s-channel%s.cap" % (datetime.now().strftime("%Hh%M.%S"), (str(channel)))
        pathname = self._device.get_report_tree().get_report_path()
        self._capture = os.path.join(pathname, filename)
        # We assume that on Windows the current drive is the ACS one
        if self._capture[1] == ":":
            # Remove the drive letter
            self._capture = self._capture[2:]

        # start capturing
        if ascii_mode:
            wifi_mac = networking_api.get_interface_mac_addr("wifi")
            self._sniffer.start_capture(channel, self._capture, True, wifi_mac, ssid)
        else:
            self._sniffer.start_capture(channel, self._capture)

        # Reset the pointer to the latest character read in the sniff file
        self._sniff_pointer = 0

#------------------------------------------------------------------------------

    def _init_connection_analysis(self, channel=None, ssid=None, ascii_mode=None, networking_api=None):
        """
        Initialize the connection analysis process
        - Start the sniffer if not already started
        - Or Update the pointer to read the sniffer log file.

        :type channel: str
        :param channel: channel in which the sniffer must be started
        :type ssid: str
        :param ssid: ssid in which the sniff should be filtered
        :type ascii_mode: str
        :param ascii_mode: Save sniffer log in ascii mode (used to be able to parse the log automatically)
        :type networking_api: Networking object
        :param networking_api: UECmd Networking API
        """
        if not self._sniffer:
            return

        # Set parameter default values
        if channel is None:
            channel = self._channel
        if ssid is None:
            ssid = self._ssid
        if ascii_mode is None:
            ascii_mode = self._bandwidth == "40" or self._wifi_mimo
        if networking_api is None:
            networking_api = self._networking_api

        if self._sniff_requested:
            # The sniffer is already running.
            # We need to forward the read pointer of the log file, in order for the next log analysis to start from now
            self.__update_current_pointer_in_sniff_file()
        else:
            # We need to start the sniffer
            self._start_sniffer(channel, ssid, ascii_mode, networking_api)

#------------------------------------------------------------------------------

    def __update_current_pointer_in_sniff_file(self):
        """
        Update the current pointer to read the sniffer log file with the end of the current file
        """
        # Retrieve the ongoing sniffer log file
        self._sniffer.get_capture_file(self._capture)
        if os.path.exists(self._capture):
            try:
                # Set the "read from here" pointer to the end of the current file
                self._sniff_pointer = os.path.getsize(self._capture)
            finally:
                # Delete the temporary file
                os.remove(self._capture)
