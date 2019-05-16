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
:summary: This file implements the LAB WIFI DUAL AP CONNECTION FAIL UC
            Connect successfully to a 1st AP,
            then try to connect to a 2nd AP with corrupted passphrase,
            then control the auto reconnection on the 1st AP,
            then connect back on the 2nd AP with the correct passphrase.
:since: 09/11/2012
:author: apairex - RTC28015
"""

from acs_test_scripts.UseCase.Networking.LAB_WIFI_DUAL_BASE import LabWifiDualBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.UECmdTypes import AUTO_CONNECT_STATE
import random
from string import hexdigits  # pylint: disable=W0402
import time
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiDualApConnectionFail(LabWifiDualBase):

    """
    Lab Wifi Connection Fail Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiDualBase.__init__(self, tc_name, global_config)

        self._wrong_passphrase = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiDualBase.set_up(self)

        # Control that the security is not OPEN
        if str(self._security_ap2).upper() in ["NONE", "OPEN"]:
            msg = "WIFI_SECURITY_AP2 UseCase parameter should not be " + \
                "undefined nor OPEN: %s" % str(self._security_ap2)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Generate a corrupted passphrase
        if self._security_ap2 == "EAP-WPA" or self._security_ap2 == "EAP-WPA2":
            self._wrong_passphrase = self._eap_method_ap2 + "-" \
                + self._phase2_auth_ap2 + "_" + self._eap_user \
                + "_" + self._corrupt_pass(self._eap_password) + "_" \
                + self._certificat_name + "_" + str(self._mandatory_cert)
        else:
            self._wrong_passphrase = self._corrupt_pass(self._passphrase_ap2)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable=E1101
        LabWifiDualBase.run_test(self)

        # Remove the 2nd SSID in known network list
        self._networking_api.wifi_remove_config(self._ssid_ap2)

        # And add the corrupted one
        self._networking_api.set_wificonfiguration(self._ssid_ap2,
                                                   self._wrong_passphrase,
                                                   self._security_ap2,
                                                   self._ip_setting_ap2,
                                                   self._ip_address_ap2,
                                                   self._netmask_ap2,
                                                   self._gateway_ap2,
                                                   self._dns1_ap2,
                                                   self._dns2_ap2)
        time.sleep(self._wait_btwn_cmd)

        # Start log monitoring
        self._networking_api.start_wifi_connection_log()

        # Try to connect with the wrong passphrase
        self._networking_api.wifi_connect(self._ssid_ap2, False)

        # Control the connection status
        connection_status = self._networking_api.\
            get_wifi_connection_status_log(self._ssid_ap2)
        self._logger.info("Connection log read: %s" % connection_status)

        if connection_status != "FAILURE":
            msg = "Unable to retrieve connection failure information from log"
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Set SSID back to ENABLE
        # (since wifi_connect automatically disables all other known SSIDs)
        self._networking_api.set_autoconnect_mode(self._ssid,
                                                  AUTO_CONNECT_STATE.on)

        # Now wait for the connection back to the 1st AP
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.check_connection_state(self._ssid)

        # Connect to the 2nd SSID with the Good passphrase
        self._networking_api.wifi_remove_config(self._ssid_ap2)
        self._networking_api.set_wificonfiguration(self._ssid_ap2,
                                                   self._passphrase_ap2,
                                                   self._security_ap2,
                                                   self._ip_setting_ap2,
                                                   self._ip_address_ap2,
                                                   self._netmask_ap2,
                                                   self._gateway_ap2,
                                                   self._dns1_ap2,
                                                   self._dns2_ap2)
        time.sleep(self._wait_btwn_cmd)
        # Connect and check the SSID connection
        self._networking_api.wifi_connect(self._ssid_ap2)

        return Global.SUCCESS, "no_error"

    def _corrupt_pass(self, passphrase):
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
