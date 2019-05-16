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

:summary: Use Case Wifi idle associated for power measurement tests.
:organization: INTEL MCG PSI
:author: dgo
:since: 15/09/2010
"""

import time

from UtilitiesFWK.Utilities import Global
from LAB_PWRMEAS_BASE import LabPwrMeasBase
from ErrorHandling.DeviceException import DeviceException


class LabPwrMeasWifiIdleAss(LabPwrMeasBase):

    """
    Use Case Wifi idle associated class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LabPwrMeasBase.__init__(self, tc_name, global_config)

        # Set "wifi" power to on in the technology_power
        # dictionnary (attribute of the power measurement base usecase)
        self._technology_power.append("wifi")

        self._sleep_mode = "s3"

        # Get WIFI Router configuration according
        # WIFI SECURITY ("OPEN", "WEP" or WPA)
        self._security = str(self._tc_parameters.
                             get_param_value("WIFI_SECURITY")).upper()

        if self._security == "WEP":
            self._logger.debug("WEP Wifi Security selected, getting parameters")
            # Get WEP WIFI Router configuration
            self._wifirouter = global_config.benchConfig.\
                get_parameters("WEP_WIFI_ROUTER")
            self._ssid = self._wifirouter.get_param_value("SSID")
            self._passphrase = self._wifirouter.get_param_value("passphrase")

        if self._security == "WPA":
            self._logger.debug("WPA Wifi Security selected, getting parameters")
            # Get WPA WIFI Router configuration
            self._wifirouter = global_config.benchConfig.\
                get_parameters("WPA_WIFI_ROUTER")
            self._ssid = self._wifirouter.get_param_value("SSID")
            self._passphrase = self._wifirouter.get_param_value("passphrase")

        if self._security in ("NONE", "OPEN"):
            self._logger.debug("\"OPEN\" Wifi Security type" +
                               " selected, getting parameters")
            self._wifirouter = \
                global_config.benchConfig.\
                get_parameters("NO_SECURITY_WIFI_ROUTER")
            self._ssid = self._wifirouter.get_param_value("SSID")

#------------------------------------------------------------------------------

    def _finalize_set_up(self):
        """
        Set up the test configuration
        """
        self._networking_api.wifi_remove_config("all")
        self._networking_api.set_wifi_power("off")
        time.sleep(5)
        self._networking_api.set_wifi_power("on")

        # Start WIFI scanning
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.request_wifi_scan()

        # Set passphrase if security is WEP or WPA
        time.sleep(self._wait_btwn_cmd)
        if self._security not in ("NONE", "OPEN"):

            self._logger.info("Setting passphrase " + str(self._passphrase) +
                              " for " + str(self._ssid) + "...")
            self._networking_api.\
                set_wificonfiguration(self._ssid,
                                      self._passphrase,
                                      self._security)

        # Connect to wifi using SSID parameter value
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.wifi_connect(self._ssid)

#------------------------------------------------------------------------------

    def _run_test_end(self):
        """
        Execute the test
        """
        # Check if wifi  is still connected
        # (blocking test for the test case verdict)
        time.sleep(self._wait_btwn_cmd)
        list_ssid = self._networking_api.list_connected_wifi()
        if len(list_ssid) == 0:
            # wifi has been disconnected
            raise DeviceException(DeviceException.CONNECTION_LOST,
                                  "Wifi has been disconnected during power measurement")

        time.sleep(self._wait_btwn_cmd)
        self._networking_api.wifi_disconnect(self._ssid)
