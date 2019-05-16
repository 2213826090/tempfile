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
:summary: This file implements LAB WIFI DUAL BASE
:since: 19/04/2012 BZ2827
:author: apairex
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global, str_to_bool_ex

import time


class LabWifiDualBase(LabWifiBase):

    """
    Lab Wifi Dual Base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Get Configurable AP #2 server parameters from Bench Config
        self._configurable_ap2 = global_config.benchConfig. \
            get_parameters("CONFIGURABLE_AP2")

        self._ssid_ap2 = self._configurable_ap2.get_param_value("SSID")
        if self._configurable_ap2.get_param_value("Model") == "AP_CONTROLLER" \
                and self._configurable_ap2.get_param_value("APC_IP", "") != "":
            self._wifirouter_ip_ap2 = \
                self._configurable_ap2.get_param_value("APC_IP")
        else:
            self._wifirouter_ip_ap2 = \
                self._configurable_ap2.get_param_value("IP")
        self._passphrase_wep64_ap2 = self._configurable_ap2. \
            get_param_value("passphrase_WEP64")
        self._passphrase_wep128_ap2 = self._configurable_ap2. \
            get_param_value("passphrase_WEP128")
        self._passphrase_wpa_ap2 = self._configurable_ap2. \
            get_param_value("passphrase_WPA")
        self._passphrase_wpa2_ap2 = self._configurable_ap2. \
            get_param_value("passphrase_WPA2")

        # Get WIFI Router configuration according WIFI_HIDDEN
        self._hidden_ap2 = self._tc_parameters.get_param_value("WIFI_HIDDEN_AP2")
        if type(self._hidden_ap2) not in [type(''), type(u'')] \
                or not str(self._hidden_ap2).isdigit():
            self._logger.warning("UC parameter 'hidden' ignored: %s."
                                 % (str(self._hidden)))
            self._hidden_ap2 = False
        else:
            if self._hidden_ap2 in (1, '1', 'on', 'ON'):
                self._hidden_ap2 = True
            else:
                self._hidden_ap2 = False

        # Get WIFI Router #2 configuration according WIFI_STANDARD
        # ("a", "b", "g", "bg, "gn", "bgn", "n2.4G", "an" or "n5G")
        self._standard_ap2 = \
            self._tc_parameters.get_param_value("WIFI_STANDARD_AP2")

        # Get WIFI Router #2 configuration according WIFI SECURITY
        self._security_ap2 = \
            self._tc_parameters.get_param_value("WIFI_SECURITY_AP2")

        # Get WIFI Router #2 configuration according WIFI CHANNEL
        self._channel_ap2 = \
            self._tc_parameters.get_param_value("WIFI_CHANNEL_AP2")
        if type(self._channel_ap2) not in [type(''), type(u'')] \
                or not self._channel_ap2.isdigit():
            self._logger.warning("UC parameter 'channel_ap2' ignored: %s."
                                 % (str(self._channel_ap2)))
            self._channel_ap2 = None

        # Get WIFI Router configuration according WIFI DTIM
        self._dtim_ap2 = self._tc_parameters.get_param_value("WIFI_DTIM_AP2")
        if type(self._dtim_ap2) not in [type(''), type(u'')] \
                or not self._dtim_ap2.isdigit():
            self._logger.warning("UC parameter 'dtim_ap2' ignored: %s."
                                 % (str(self._dtim_ap2)))
            self._dtim_ap2 = None

        # Get WIFI Router configuration according WIFI BEACON
        self._beacon_ap2 = self._tc_parameters.get_param_value("WIFI_BEACON_AP2")
        if type(self._beacon_ap2) not in [type(''), type(u'')] \
                or not self._beacon_ap2.isdigit():
            self._logger.warning("UC parameter 'beacon' ignored: %s."
                                 % (str(self._beacon_ap2)))
            self._beacon_ap2 = None

        # Get WIFI Router configuration according WIFI WMM
        self._wmm_ap2 = self._tc_parameters.get_param_value("WIFI_WMM_AP2")
        if type(self._wmm_ap2) not in [type(''), type(u'')] \
                or not self._wmm_ap2.isdigit():
            self._logger.warning("UC parameter 'wmm_ap2' ignored: %s."
                                 % (str(self._wmm_ap2)))
            self._wmm_ap2 = None

        # Get WIFI Router configuration according WIFI BAND
        self._bandwidth_ap2 = self._tc_parameters.get_param_value("WIFI_BAND_AP2")
        if self._bandwidth_ap2 == "":
            self._logger.warning("UC parameter 'bandwidth_ap2' ignored: %s." % (str(self._bandwidth_ap2)))
            self._bandwidth_ap2 = None

        # Get the WIFI MIMO activation toggle
        self._wifi_mimo_ap2 = str_to_bool_ex(self._tc_parameters.get_param_value("WIFI_MIMO_AP2", "False"))

        self._eap_method_ap2 = None
        self._phase2_auth_ap2 = None
        self._passphrase_ap2 = None
        if self._security_ap2 in ["WEP64", "WEP64-OPEN"]:
            self._passphrase_ap2 = self._passphrase_wep64_ap2
        elif self._security_ap2 in ["WEP128", "WEP128-OPEN"]:
            self._passphrase_ap2 = self._passphrase_wep128_ap2
        elif self._security_ap2 == "WPA-PSK-TKIP" or \
                self._security_ap2 == "WPA-PSK-AES":
            self._passphrase_ap2 = self._passphrase_wpa_ap2
        elif self._security_ap2 == "WPA2-PSK-AES" or \
            self._security_ap2 == "WPA2-PSK-TKIP" or \
                self._security_ap2 == "WPA2-PSK-TKIP-AES":
            self._passphrase_ap2 = self._passphrase_wpa2_ap2
        elif self._security_ap2 == "EAP-WPA" \
                or self._security_ap2 == "EAP-WPA2":

            # Get WIFI Router configuration according to WPA-enterprise options
            self._eap_method_ap2 = str(self._tc_parameters.
                                       get_param_value("EAP_METHOD_AP2"))
            self._phase2_auth_ap2 = str(self._tc_parameters.
                                        get_param_value("PHASE2_AUTH_AP2"))

            # Compute passphrase for WPA-ENT (will be parsed by ACS Embedded)
            self._passphrase_ap2 = self._eap_method_ap2 + "-" \
                + self._phase2_auth_ap2 \
                + "_" + self._eap_user + "_" + self._eap_password \
                + "_" + self._certificat_name

        # Get DUT static ip configuration according IP_SETTING for AP2
        self._ip_setting_enable_ap2 = str_to_bool_ex(self._tc_parameters.get_param_value("IP_SETTING_ENABLE_AP2"))
        self._ip_address_ap2 = str(self._tc_parameters.get_param_value("IP_ADDRESS_AP2"))
        self._netmask_ap2 = str(self._tc_parameters.get_param_value("NETMASK_AP2"))
        self._gateway_ap2 = str(self._tc_parameters.get_param_value("GATEWAY_AP2"))
        self._dns1_ap2 = str(self._tc_parameters.get_param_value("DNS1_AP2"))
        self._dns2_ap2 = str(self._tc_parameters.get_param_value("DNS2_AP2"))
        # DHCP or STATIC ip settings
        if self._ip_setting_enable_ap2:
            self._ip_setting_ap2 = "static"
        else:
            self._ip_setting_ap2 = "dhcp"

        # initial wifi equipment for access point #2
        self._ns_ap2 = self._em.get_configurable_ap("CONFIGURABLE_AP2")

        # Load sniffer equipment if not already done
        if self._bandwidth_ap2 == "40" and self._sniffer is None:
            self._sniffer = self._em.get_sniffer("WIFI_SNIFFER1")

        self._sniff_paused = False
        self._same_ssid = False

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        if self._sniff_requested:
            self._pause_sniffer()

        self._configure_ap2()

        # Extract the security Type
        if self._bandwidth_ap2 == "40" or self._sniff_requested:
            self._start_sniffer(self._channel_ap2, self._ssid_ap2, self._bandwidth_ap2 == "40")

        if not self._same_ssid:
            self.__set_dut_configuration(self._ssid_ap2,
                                        self._passphrase_ap2,
                                        self._security_ap2,
                                        self._ip_setting_ap2,
                                        self._ip_address_ap2,
                                        self._netmask_ap2,
                                        self._gateway_ap2,
                                        self._dns1_ap2,
                                        self._dns2_ap2)
        else:
            # Get the DUT MAC address used for connection check (only in 'same SSID' case)
            self._dut_mac_address_for_connection_check = \
                self._networking_api.get_interface_mac_addr("wifi").lower()

        if self._bandwidth_ap2 == "40":
            # disable ap1 and check connection to ap2
            if self._same_ssid:
                self._ns.init()
                self._ns.disable_wireless()
                self._ns.release()

            self._networking_api.wifi_connect(self._ssid_ap2, False)
            self._check_40mhz_connection()

            if self._same_ssid:
                # disable ap 2 then reconnect to ap1
                self._ns_ap2.init()
                self._ns_ap2.disable_wireless()
                self._ns_ap2.release()
                # enable ap1
                self._ns.init()
                self._ns.enable_wireless()
                self._ns.release()

        self._resume_sniffer()
        # connect to ap1 if required
        if self._connect_to_ap1 and self._bandwidth_ap2 == "40":
            self._networking_api.wifi_connect(self._ssid, False)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # stop sniffing
        if self._sniffer:
            if self._sniff_requested and self._capture:
                self._sniffer.stop_capture(self.test_ok)
            self._sniffer.release()

        self._networking_api.wifi_remove_config(self._ssid)
        time.sleep(self._wait_btwn_cmd)

        self._networking_api.wifi_remove_config(self._ssid_ap2)
        time.sleep(self._wait_btwn_cmd)

        self._networking_api.set_wifi_power("off")

        # Disable radio for all Wifi APs
        self._disable_all_radios()

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            bt_final_status = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the initial value, set it to the correct one.
            if bt_final_status != self._bt_initial_state:
                # restore Bt in initial state
                self._localconnectivity_api.set_bt_power(self._bt_initial_state)
                time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

    def _configure_ap2(self):
        """
        Configure AP #2
        """
        # Initiate connection to the secondary Access Point equipment
        self._ns_ap2.init()

        # Configure the AP #2 equipment
        self._ns_ap2.set_wifi_config(self._ssid_ap2,
                                     self._hidden_ap2,
                                     self._standard_ap2,
                                     self._security_ap2,
                                     self._passphrase_ap2,
                                     self._channel_ap2,
                                     self._dtim_ap2,
                                     self._beacon_ap2,
                                     self._wmm_ap2,
                                     self._bandwidth_ap2,
                                     self._wifi_mimo_ap2,
                                     self._radiusip,
                                     self._radiusport,
                                     self._radiussecret)
        self._ns_ap2.enable_wireless()

        if self._sniffer and self._channel_ap2 is None:
                # If using auto channel, retrieve the selected channel from the AP
                self._channel_ap2 = self._ns_ap2.get_selected_channel()
        # Close the connection to the AP2
        self._ns_ap2.release()

    def _pause_sniffer(self):
        """
        Pause the sniffer and save a copy of the current sniff locally
        """
        self._sniffer.stop_capture(False)
        self._sniff_paused = True

    def _resume_sniffer(self):
        """
        restart the sniffer as the user requested it
        """
        # build a unique capture file name with a timestamp inside
        # the Campaign Report directory
        if self._sniff_paused:
            # stop sniffing on ap2 channel an restart sniffing on ap1 channel
            self._sniffer.stop_capture(True)
            self._sniff_paused = False
            self._start_sniffer(self._channel, self._ssid, self._bandwidth == "40")

    def __set_dut_configuration(self,
                               ssid,
                               passphrase,
                               security,
                               ip_setting,
                               ip_address,
                               netmask,
                               gateway,
                               dns1,
                               dns2):
        # configure DUT with ap parameters
        self._networking_api.set_wificonfiguration(ssid,
                                                   passphrase,
                                                   security,
                                                   ip_setting,
                                                   ip_address,
                                                   netmask,
                                                   gateway,
                                                   dns1,
                                                   dns2)
        # Need to restart Wifi interface to take into account enabled SSID
        self._networking_api.set_wifi_power(0)
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_power(1)
        time.sleep(self._wait_btwn_cmd)
