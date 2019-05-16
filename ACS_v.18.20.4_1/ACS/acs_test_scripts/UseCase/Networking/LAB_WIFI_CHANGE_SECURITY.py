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
:summary: This file implements the LIVE WIFI CONNECT UC
:since: 07/07/2014
:author: jfranchx
"""

import time
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase


class LabWifiChangeSecurity(LabWifiBase):
    """
    Lab Wifi Change Security Test class.
    """

    WAIT_FOR_DISCONNECTION = 5.0
    PING_PACKET_SIZE = 16
    PING_PACKET_COUNT = 4

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        self._passphrase_2 = None
        self._security_2 = str(self._tc_parameters.get_param_value("WIFI_SECURITY_2"))
        self._eap_user_2 = self._tc_parameters.get_param_value("EAP_USER", self.__get_bc_ap_param("EAP_user"))
        self._eap_password_2 = self.__get_bc_ap_param("EAP_password")

        if self._security_2 in ["WEP64", "WEP64-OPEN"]:
            self._passphrase_2 = self._passphrase_wep64
        elif self._security_2 in ["WEP128", "WEP128-OPEN"]:
            self._passphrase_2 = self._passphrase_wep128
        elif self._security_2 in ["EAP-WPA", "EAP-WPA2"]:
            # Get WIFI Router configuration according to WPA-enterprise options
            self._eap_method_2 = str(self._tc_parameters.get_param_value("EAP_METHOD_2"))
            self._phase2_auth_2 = str(self._tc_parameters.get_param_value("PHASE2_AUTH_2"))
            if self._eap_method_2 == "SIM":
                self._eap_user_2 = self.__get_bc_ap_param("EAP_SIM_user")
                # If no username is required, we don't need the password
                if self._eap_user_2 == "":
                    self._eap_password = ""
            elif self._eap_method_2 == "AKA":
                self._eap_user_2 = self.__get_bc_ap_param("EAP_AKA_user")
                # If no username is required, we don't need the password
                if self._eap_user_2 == "":
                    self._eap_password_2 = ""
            # Compute passphrase for WPA-ENT (will be parsed by ACS Embedded)
            self._passphrase_2 = self._eap_method_2 + "-" + self._phase2_auth_2 \
                                 + "_" + self._eap_user_2 + "_" + self._eap_password_2 \
                                 + "_" + self._certificat_name + "_" + str(self._mandatory_cert)
        elif "WPA2" in self._security_2:
            self._passphrase_2 = self._passphrase_wpa2
        elif "WPA" in self._security_2:
            self._passphrase_2 = self._passphrase_wpa

        #------------------------------------------------------------------------------

    def set_up(self):
        """
        Setup the test
        """
        LabWifiBase.set_up(self)

        # Check if the two security are different, otherwise the test is useless
        if self._security == self._security_2:
            msg = "The two securities given are identical : %s ! They must be different " % str(self._security_2)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "no_error"

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Change AP security type
        # Initiate connection to the equipment
        self._ns.init()
        try:
            # Configure the equipment
            self._ns.set_wifi_config(self._ssid,
                                     self._hidden,
                                     self._standard,
                                     self._security_2,
                                     self._passphrase_2,
                                     self._channel,
                                     self._dtim,
                                     self._beacon,
                                     self._wmm,
                                     self._bandwidth,
                                     self._wifi_mimo,
                                     self._radiusip,
                                     self._radiusport,
                                     self._radiussecret)
        finally:
            # Close the connection to AP
            self._ns.release()

        self._logger.debug("Wait %ss for automatic disconnection" % self.WAIT_FOR_DISCONNECTION)
        time.sleep(self.WAIT_FOR_DISCONNECTION)

        # Ping to validate DUT is not connected
        packet_loss = self._networking_api.ping(self._wifi_server_ip_address, self.PING_PACKET_SIZE, self.PING_PACKET_COUNT,
                                                blocking=False)
        if packet_loss.value < 100 and packet_loss.value != -1:
            msg = "Ping packet loss is not acceptable [%s] - DUT should be disconnected" % str(packet_loss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Connect with good credentials
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wificonfiguration(self._ssid,
                                                   self._passphrase_2,
                                                   self._security_2,
                                                   self._ip_setting,
                                                   self._ip_address,
                                                   self._netmask,
                                                   self._gateway,
                                                   self._dns1,
                                                   self._dns2)

        time.sleep(self._wait_btwn_cmd)
        self._networking_api.wifi_connect(self._ssid)

        # Ping to validate DUT is connected
        time.sleep(self._wait_btwn_cmd)
        packet_loss = self._networking_api.ping(self._wifi_server_ip_address, self.PING_PACKET_SIZE, self.PING_PACKET_COUNT)
        if packet_loss.value > 0:
            msg = "Ping packet loss is not acceptable [%s] - DUT should be connected" % str(packet_loss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "no_error"

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
            default_value = self._configurable_default_ap. \
                get_param_value(param)

        # Retrieve the bench config value
        value = self._configurable_ap.get_param_value(param, default_value)

        return value
