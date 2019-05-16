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
:summary: This file implements the LAB_WIFI_DEBUG_TEST
:author: apairex
:since:27/03/2013
"""
import time
import os
from LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.Misc.UECMD_TEST_TOOL import UECmdTestTool
from acs_test_scripts.Device.UECmd.UECmdTypes import AUTO_CONNECT_STATE
from acs_test_scripts.Utilities.IPerfUtilities import parse_iperf_options
from ErrorHandling.AcsConfigException import AcsConfigException


class LabWifiDebugTest(LabWifiBase, UECmdTestTool):

    """
    Lab WiFi TEST UECmd UseCase
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LabWifiBase init function
        LabWifiBase.__init__(self, tc_name, global_config)
        UECmdTestTool.__init__(self, self.get_name(),
                               self._logger, self._device)

        self._run_basic = self._tc_parameters.get_param_value("RUN_BASIC_TEST", "")
        self._run_connect = self._tc_parameters.get_param_value("RUN_CONNECT_TEST", "")
        self._run_net = self._tc_parameters.get_param_value("RUN_NET_TEST", "")
        self._run_data = self._tc_parameters.get_param_value("RUN_DATA_TEST", "")
        self._run_other = self._tc_parameters.get_param_value("RUN_OTHER_TEST", "")
        self._run_wps = self._tc_parameters.get_param_value("RUN_WPS_TEST", "")
        self._run_ipv6 = self._tc_parameters.get_param_value("RUN_IPV6_TEST", "")
        if self._run_basic.lower() in ["1", "on", "yes"]:
            self._run_basic = "true"
        if self._run_connect.lower() in ["1", "on", "yes"]:
            self._run_connect = "true"
        if self._run_net.lower() in ["1", "on", "yes"]:
            self._run_net = "true"
        if self._run_data.lower() in ["1", "on", "yes"]:
            self._run_data = "true"
        if self._run_other.lower() in ["1", "on", "yes"]:
            self._run_other = "true"
        if self._run_wps.lower() in ["1", "on", "yes"]:
            self._run_wps = "true"
        if self._run_ipv6.lower() in ["1", "on", "yes"]:
            self._run_ipv6 = "true"

        self._ul_filename = self._tc_parameters.get_param_value("FTP_UL_FILE", "")
        self._dl_filename = self._tc_parameters.get_param_value("FTP_DL_FILE", "")
        self._iperf_options = self._tc_parameters.get_param_value("IPERF_OPTIONS", "")
        self._webste_url = self._tc_parameters.get_param_value("WEBSITE_URL", "")

        self._basic = list()
        self._connect = list()
        self._net = list()
        self._data = list()
        self._other = list()
        self._wps = list()
        self._ipv6 = list()

        # Ignore IP configuration settings handled in LAB_WIFI_BASE
        self._ip_setting = "dhcp"
        self._ip_address = ""
        self._netmask = ""
        self._gateway = ""
        self._dns1 = ""
        self._dns2 = ""

        # Ignore other optional LAB_WIFI_BASE parameters
        self._hidden = False
        self._channel = None
        self._dtim = None
        self._beacon = None
        self._wmm = None
        self._bandwidth = None
        self._use_flight_mode = 0
        self._user_reg_domain = None
        self._key_exchange_should_fail = False
        self._bt_api = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)
        UECmdTestTool.set_up(self, self.tc_order)

        self._bt_api = self._device.get_uecmd("LocalConnectivity")

        self._run_basic = str_to_bool(str(self._run_basic))
        self._run_connect = str_to_bool(str(self._run_connect))
        self._run_net = str_to_bool(str(self._run_net))
        self._run_data = str_to_bool(str(self._run_data))
        self._run_other = str_to_bool(str(self._run_other))
        self._run_wps = str_to_bool(str(self._run_wps))
        self._run_ipv6 = str_to_bool(str(self._run_ipv6))

        if self._run_data and (self._dl_filename == "" or self._ul_filename == ""):
            msg = "Wrong DL_FILE or UL_FILE TC parameter: DL: %s - UL: %s" \
                % (self._dl_filename, self._ul_filename)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._run_data and self._iperf_options == "":
            msg = "Empty IPERF_OPTIONS tc parameters"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # WiFi Connection parameters
        connection_param = [self._ssid,
                            self._passphrase,
                            self._security,
                            self._ip_setting,
                            self._ip_address,
                            self._netmask,
                            self._gateway,
                            self._dns1,
                            self._dns2]

        # EAP-WPA certificate parameters
#        certif_param = [self._configurable_ap.get_param_value("CA_guard_password"), \
#                        self._configurable_ap.get_param_value("EAP_password"), \
#                        self._configurable_ap.get_param_value("Credential_password")]

        # Data transfer test parameters
        server_ip_address = self._wifi_server_ip_address
        iperf_settings = {"server_ip_address": server_ip_address,
                          "mode": "single", "direction": "up",
                          "": self._iperf_options}
        iperf_settings.update(parse_iperf_options(self._iperf_options))

        # FTP parameters
        dlfilename = os.path.join(self._ftp_path, self._dl_filename)
        ulfilename = os.path.join(self._ftp_path, self._ul_filename)
        ftp_dl_param = [self._uecmd_types.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                        self._ftp_ip_address,
                        self._ftp_username,
                        self._ftp_password,
                        dlfilename,
                        120,
                        self._device.get_ftpdir_path()]
        ftp_ul_param = [self._uecmd_types.XFER_DIRECTIONS.UL,  # pylint: disable=E1101
                        self._ftp_ip_address,
                        self._ftp_username,
                        self._ftp_password,
                        ulfilename,
                        120,
                        self._device.get_ftpdir_path()]

        # Tethering parameter
        soft_ap_param = ["on", "cws_test_debug", "WPA2-PSK", "acdc123456"]

        # Wifi sleep policy parameters
        sleep_policy_param = self._networking_api.WIFI_SLEEP_POLICY["DEFAULT"]
        wifi_off_duration_param = ["async_init", self._dut_wlan_iface, 3]

        # shortcut to the networking API
        net_api = self._networking_api

        """
        List of tests to perform. Each test is described as followed:
        [Label to print in the secondary report,
         UECmd to test,
         parameter(s) for the UECmd to test,
         Depends on the test names in the list]
        """
        # pylint: disable=C0301
        if self._run_basic:
            self._basic = [
                {self._NAME: "set_wifi_power OFF", self._FCT: net_api.set_wifi_power, self._PARMS: ['off']},
                {self._NAME: "set_wifi_power ON", self._FCT: net_api.set_wifi_power, self._PARMS: ['on']},
                {self._FCT: net_api.get_wifi_power_status},
                {self._FCT: net_api.get_wifi_power_level},
                {self._NAME: "set_wifi_power_saving_mode ON", self._FCT: net_api.set_wifi_power_saving_mode, self._PARMS: ["on"]},
                {self._NAME: "set_wifi_power_saving_mode OFF", self._FCT: net_api.set_wifi_power_saving_mode, self._PARMS: ["off"]},
                {self._NAME: "get_interface_mac_addr", self._FCT: net_api.get_interface_mac_addr},
                # {self._NAME: "set_mac_addr", self._FCT: self.__set_mac_addr, self._DEP: ["get_interface_mac_addr"]},
                {self._FCT: net_api.get_wifi_ip_address},
                {self._FCT: net_api.get_interface_ipv4_address, self._PARMS: [self._dut_wlan_iface]}
                # {self._FCT: net_api.load_wpa_certificate, self._PARMS: certif_param}
            ]

        if self._run_connect:
            self._connect = [
                # pylint: disable=E1101
                {self._FCT: net_api.request_wifi_scan},
                {self._NAME: "set_autoconnect_mode ON", self._FCT: net_api.set_autoconnect_mode, self._PARMS: [self._ssid, AUTO_CONNECT_STATE.on]},
                {self._FCT: net_api.wifi_remove_config, self._PARMS: [self._ssid]},
                {self._FCT: net_api.check_ssid_bfore_timeout, self._PARMS: [self._ssid, 5]},
                {self._FCT: net_api.set_wificonfiguration, self._PARMS: connection_param},
                {self._FCT: net_api.start_wifi_connection_log, self._PARMS: []},
                {self._FCT: net_api.wifi_connect, self._PARMS: [self._ssid, False]},
                {self._FCT: net_api.get_wifi_connection_status_log, self._PARMS: [self._ssid, 15]},
                {self._FCT: net_api.list_connected_wifi},
                {self._FCT: net_api.list_ssids},
                {self._FCT: net_api.list_ssids_and_capabilities},
                {self._FCT: net_api.list_ssids_and_frequency},
                {self._FCT: net_api.check_connection_state, self._PARMS: [self._ssid]},
                {self._FCT: net_api.get_wifi_dhcp_state},
                {self._FCT: net_api.wait_for_wifi_dhcp_connected},
                {self._FCT: net_api.wifi_disconnect, self._PARMS: [self._ssid]},
                {self._FCT: net_api.wifi_disconnect_all}
            ]

        if self._run_net:
            self._net = [
                {self._FCT: net_api.activate_pdp_context},
                {self._FCT: net_api.deactivate_pdp_context},
                {self._NAME: "set_flight_mode ON", self._FCT: net_api.set_flight_mode, self._PARMS: ['on']},
                {self._FCT: net_api.get_flight_mode},
                {self._NAME: "set_flight_mode OFF", self._FCT: net_api.set_flight_mode, self._PARMS: ['off']},
                {self._FCT: net_api.clean_all_data_connections}
            ]

        if self._run_data:
            self._data = [
                {self._NAME: "set_wifi_power ON DATA", self._FCT: net_api.set_wifi_power, self._PARMS: ['on']},
                {self._NAME: "set_wificonfiguration DATA", self._FCT: net_api.set_wificonfiguration, self._PARMS: connection_param,
                 self._DEP: ["set_wifi_power ON DATA"]},
                {self._NAME: "wifi_connect DATA", self._FCT: net_api.wifi_connect, self._PARMS: [self._ssid, False],
                 self._DEP: ["set_wificonfiguration DATA"]},
                {self._FCT: net_api.iperf, self._PARMS: [iperf_settings],
                 self._DEP: ["wifi_connect DATA"]},
                {self._FCT: net_api.ping, self._PARMS: [server_ip_address, 128, 2],
                 self._DEP: ["wifi_connect DATA"]},
                {self._NAME: "ftp_xfer DL", self._FCT: net_api.ftp_xfer, self._PARMS: ftp_dl_param,
                 self._DEP: ["wifi_connect DATA"]},
                {self._NAME: "ftp_xfer UL", self._FCT: net_api.ftp_xfer, self._PARMS: ftp_ul_param,
                 self._DEP: ["wifi_connect DATA"]},
                {self._NAME: "wifi_disconnect_all DATA", self._FCT: net_api.wifi_disconnect_all},
                {self._NAME: "activate_pdp_context DATA", self._FCT: net_api.activate_pdp_context},
                {self._NAME: "open_web_browser native", self._FCT: net_api.open_web_browser, self._PARMS: [self._webste_url, "native"],
                 self._DEP: ["activate_pdp_context DATA", "wifi_disconnect_all DATA"]},
                {self._NAME: "close_web_browser native", self._FCT: net_api.close_web_browser, self._PARMS: ["native"],
                 self._DEP: ["open_web_browser native"]},
                {self._NAME: "open_web_browser ACS", self._FCT: net_api.open_web_browser, self._PARMS: [self._webste_url, "acs_agent"],
                 self._DEP: ["activate_pdp_context DATA", "wifi_disconnect_all DATA"]},
                {self._NAME: "close_web_browser ACS", self._FCT: net_api.close_web_browser, self._PARMS: ["acs_agent"],
                 self._DEP: ["open_web_browser ACS"]}
            ]

        if self._run_other:
            self._other = [
                {self._NAME: "set_wifi_power ON OTHER", self._FCT: net_api.set_wifi_power, self._PARMS: ['on']},
                {self._FCT: net_api.set_wifi_sleep_policy, self._PARMS: [sleep_policy_param]},
                {self._FCT: net_api.get_wifi_sleep_policy},
                {self._FCT: net_api.wifi_menu_settings, self._PARMS: [True]},
                {self._FCT: net_api.get_wifi_frequency_band, self._PARMS: [False, self._dut_wlan_iface]},
                {self._FCT: net_api.set_regulatorydomain, self._PARMS: self._dut_wlan_iface},
                {self._FCT: net_api.get_regulatorydomain},
                {self._NAME: "set_wifi_hotspot ON", self._FCT: net_api.set_wifi_hotspot, self._PARMS: soft_ap_param},
                {self._NAME: "set_wifi_hotspot OFF", self._FCT: net_api.set_wifi_hotspot, self._PARMS: ["off"]},
                {self._FCT: net_api.usb_tether, self._PARMS: [True, False, False]},
                {self._FCT: net_api.measure_wifi_turns_off_duration, self._PARMS: wifi_off_duration_param},
                # {self._FCT: net_api.redirect_log_on_dut},
                {self._FCT: net_api.kill_log_on_dut}
                # Not testable yet
                # {self._FCT: net_api.retrieve_dhcp_renewal_interval_on_dut, self._PARMS: [2]},
                # {self._FCT: net_api.retrieve_dhcp_renewal_interval, self._PARMS: [2, 15]}
            ]

        if self._key_exchange_mode is not None and self._run_wps:
            self._wps = [
                {self._NAME: "wifi_setkeyexchange", self._FCT: self.__wifi_setkeyexchange}
            ]

        if self._use_ipv6 and self._run_ipv6:
            self._ipv6 = [
                # Not used
                # {self._FCT: net_api.get_interface_ipv6_address},
                # {self._FCT: net_api.get_interface_ipv6_all_address},
                {self._FCT: net_api.launch_dhcp6_client, self._PARMS: ["STATELESS_RDNSSD", self._dut_wlan_iface]},
                {self._FCT: net_api.check_ipv6_consistency, self._PARMS: [self._dut_wlan_iface, "2001:1234:5678:9ABC"]},
                {self._FCT: net_api.copy_dhcp6_dns, self._PARMS: [self._dut_wlan_iface, '1', True]},
                {self._FCT: net_api.launch_rdnssd_client, self._PARMS: [self._dut_wlan_iface]},
                # {self._FCT: net_api.ping6},
                {self._FCT: net_api.enable_ip_filtering, self._PARMS: [self._wlan_phy_device]},
                {self._FCT: net_api.disable_ip_filtering, self._PARMS: [self._wlan_phy_device]}
            ]

# Not used:
# {self._FCT: net_api.set_wifi_scanning},
# {self._FCT: net_api.get_wifi_scanning_status},
# {self._FCT: net_api.wifi_set_scan_interval},

# Used out of CWS scope:
# {self._FCT: net_api.activate_pdp_context},
# {self._FCT: net_api.deactivate_pdp_context},
# {self._FCT: net_api.set_flight_mode, self._PARMS: ['on']},
# {self._FCT: net_api.get_flight_mode},
# {self._FCT: net_api.get_available_technologies, self._PARMS: connection_param},

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Check every EUCmd
        for fct2test in self._basic \
            + self._connect \
            + self._net \
            + self._data \
            + self._other \
            + self._wps \
                + self._ipv6:
            self._check_uecmd(fct2test)
            time.sleep(self._wait_btwn_cmd)

        # Raise an Exception in case of all tests do not pass
        self._compute_general_verdict()

        return Global.SUCCESS, "All UECmds OK"

#------------------------------------------------------------------------------

    def __set_mac_addr(self):
        """
        Sub function to test write MAC address UECmd
        """
        mac = self._networking_api.get_interface_mac_addr()
        time.sleep(self._wait_btwn_cmd)

        # Write the same MAC address
        self._bt_api.set_mac_addr("wifi", mac)

    def __wifi_setkeyexchange(self):
        """
        Sub function to test WPS UECmd
        """
        # Set equipment part
        self._ns.init()
        try:
            self._ns.set_wifi_key_exchange(self._key_exchange_mode,
                                           self._key_exchange_pin)
        finally:
            self._ns.release()

        # Set DUT part
        self._networking_api.wifi_setkeyexchange(self._ssid,
                                                 self._key_exchange_mode,
                                                 self._key_exchange_pin,
                                                 self._key_exchange_should_fail,
                                                 self._dut_wlan_iface)
