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
:summary: This file implements the LAB WIFI CONNECTION FAIL UC
            Test Wifi connection with corrupted passphrase fails
            and then wifi connection with good passphrase passes
:since: 06/11/2012
:author: apairex - RTC28013
"""

from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
import random
from string import hexdigits  # pylint: disable=W0402
import time
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiConnectionFail(LabWifiBase):

    """
    Lab Wifi Connection Fail Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Read time to wait between tests
        self._time2wait = self._tc_parameters.\
            get_param_value("TIME_BETWEEN_TESTS")

        self._wrong_passphrase = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        # Control testcase parameter
        if str(self._time2wait).isdigit():
            self._time2wait = int(self._time2wait)
        else:
            msg = "TIME_BETWEEN_TESTS is missing or wrong: %s" \
                % str(self._time2wait)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Control that the security is not OPEN
        if str(self._security).upper() in ["NONE", "OPEN"]:
            msg = "WIFI_SECURITY UseCase parameter should not be " + \
                "undefined nor OPEN: %s" % str(self._security)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Generate a corrupted passphrase
        if self._security == "EAP-WPA" or self._security == "EAP-WPA2":
            self._wrong_passphrase = self._eap_method + "-" \
                + self._phase2_auth + "_" + self._corrupt(self._eap_user) \
                + "_" + self._corrupt(self._eap_password) + "_" \
                + self._corrupt(self._certificat_name) + "_" \
                + str(self._mandatory_cert)
        else:
            self._wrong_passphrase = self._corrupt(self._passphrase)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Wifi disconnection
        self._networking_api.wifi_remove_config(self._ssid)
        self._logger.info("Waiting %s seconds" % str(self._time2wait))
        time.sleep(self._time2wait)

        # Start log monitoring
        self._networking_api.start_wifi_connection_log()

        # Try to connect with the wrong passphrase
        self._networking_api.set_wificonfiguration(self._ssid,
                                                   self._wrong_passphrase,
                                                   self._security,
                                                   self._ip_setting,
                                                   self._ip_address,
                                                   self._netmask,
                                                   self._gateway,
                                                   self._dns1,
                                                   self._dns2)
        self._networking_api.wifi_connect(self._ssid, False)

        # Control the connection status
        connection_status = self._networking_api.\
            get_wifi_connection_status_log(self._ssid)
        self._logger.info("Connection log read: %s" % connection_status)

        if connection_status != "FAILURE":
            msg = "Unable to retrieve connection failure information from log"
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Wifi disconnection
        self._networking_api.wifi_remove_config(self._ssid)
        self._logger.info("Waiting %s seconds" % str(self._time2wait))
        time.sleep(self._time2wait)

        # Now try to connect with the real passphrase
        # Start log monitoring
        self._networking_api.start_wifi_connection_log()
        self._networking_api.set_wificonfiguration(self._ssid,
                                                   self._passphrase,
                                                   self._security,
                                                   self._ip_setting,
                                                   self._ip_address,
                                                   self._netmask,
                                                   self._gateway,
                                                   self._dns1,
                                                   self._dns2)
        self._networking_api.wifi_connect(self._ssid, True)
        # Control the connection status
        connection_status = self._networking_api.\
            get_wifi_connection_status_log(self._ssid)
        self._logger.info("Connection log read: %s" % connection_status)

        return Global.SUCCESS, "no_error"

    def _corrupt(self, passphrase):
        """
        Generate a corrupted passphrase by changing all characters that
        are the same as the 1st one in the passphrase by another random one

        :type passphrase: str
        :param passphrase: word to corrupt

        :rtype: str
        :return: corrupted passphrase
        """
        first_char = passphrase[0]
        new_char = first_char
        while new_char == first_char:
            new_char = random.choice(hexdigits)
        return passphrase.replace(first_char, new_char)
