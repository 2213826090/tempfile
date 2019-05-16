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
:summary: This file implements the LAB WIFI CRDA UseCase.
Based on CRDA settings, check the channel availability for a given regulatory domain
:since: 15/04/2013
:author: apairex
"""

from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies
from UtilitiesFWK.Utilities import Global, str_to_bool

import time
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiCrda(LabWifiBase):

    """
    Lab Wifi Connect Test class.
    """

    __CONNECTION_RATE_FOR_B_STANDARD = 11

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        self._connection_expected = self._tc_parameters.get_param_value("CONNECTION_EXPECTED")
        self._dut_crda = self._tc_parameters.get_param_value("DUT_REGULATORY_DOMAIN")

        # Disable WRONG_PASSPHRASE TC parameter
        self._wrong_passphrase = None
        # Disable REGULATORY_DOMAIN TC parameter
        self._user_reg_domain = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Check mandatory parameter
        if self._channel is None or int(self._channel) <= 0:
            msg = "CHANNEL TC parameter is not defined: " + str(self._channel)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Specific case for channel 14 in Japan
        if int(self._channel) == 14 and self._dut_crda == "JP" \
                and self._standard.lower() != "b" \
                and not self._connection_expected:
            # this case is prohibited
            msg = "For Channel 14 in Japan, "
            msg += "TC parameter CONNECTION_EXPECTED should be set to True"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Set AP CRDA
        self._set_ap_generic_crda(self._channel)

        # Call LabWifiBase setup without connection
        LabWifiBase.set_up_without_connect(self)

        self._connection_expected = str_to_bool(str(self._connection_expected))

        # Set DUT regulatory domain
        self._networking_api.set_regulatorydomain(self._dut_crda, self._dut_wlan_iface)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "no_error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Start log monitoring
        self._networking_api.start_wifi_connection_log()

        # Try to connect to the WiFi network
        self._networking_api.wifi_connect(self._ssid, self._connection_expected)

        # Control the connection status
        connection_status = self._networking_api.\
            get_wifi_connection_status_log(self._ssid)
        self._logger.info("Connection log read: %s" % connection_status)

        if self._connection_expected and connection_status != "SUCCESS":
            msg = "Unable to connect to channel %s while DUT CRDA is %s" \
                % (str(self._channel), self._dut_crda)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not self._connection_expected and connection_status == "SUCCESS":
            # disconnect
            self._networking_api.wifi_disconnect(self._ssid)

            # Raise an Error
            msg = "Successfully connected to channel %s while DUT CRDA is %s" \
                % (str(self._channel), self._dut_crda)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Specific case for channel 14 in Japan
        if int(self._channel) == 14 and self._dut_crda == "JP" \
                and self._standard.lower() != "b":
            # Check connection rate corresponds to B standard (11 MByte/s)
            connection_rate = self._networking_api.\
                get_wifi_connection_rate(self._dut_wlan_iface)
            self._logger.info("Connection rate: %d MB/s" % connection_rate)

            if connection_rate != self.__CONNECTION_RATE_FOR_B_STANDARD:
                msg = "Channel 14 in Japan connection rate test failed."
                msg += " connection_rate read: " + str(connection_rate)
                self._logger.error(msg)
                # Disconnect WiFi
                self._networking_api.wifi_disconnect(self._ssid)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Disconnect WiFi
        if self._connection_expected:
            self._networking_api.wifi_disconnect(self._ssid)

        return Global.SUCCESS, "no_error"

#------------------------------------------------------------------------------

    def _set_ap_generic_crda(self, channel):
        """
        Set the regulatory domain for the AP that will allow channel # to be set

        :type channel: str or int
        :param channel: channel number for the AP. Used to know the used band.
        """
        if not self._ns.is_regulatorydomain_configurable:
            msg = "YOU CANNOT CONFIGURE CRDA ON ACCESS POINT OF THE BENCH."
            msg += " You may not be able to run all CRDA tests."
            self._logger.warning(msg)
            return

        if int(channel) in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G:
            # Japan is using all available frequencies
            crda = "JP"
        elif int(channel) in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G:
            # Brazil is using all available frequencies
            crda = "BR"
        else:
            msg = "Unavailable CHANNEL TC parameter: " + str(channel)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        self._ns.set_regulatorydomain(crda)
