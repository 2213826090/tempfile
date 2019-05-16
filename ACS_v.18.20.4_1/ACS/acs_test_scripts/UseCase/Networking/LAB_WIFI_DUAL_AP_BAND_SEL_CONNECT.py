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
:summary: This file implements the LAB WIFI DUAL AP BAND SELECTION CONNECTION
        This UseCase configures 2 APs: 1 in 5GHz, and 1 in 2.4GHz.
        Band Selection option is changing during run_test and we check that
        the phone is automatically reconnecting to the visible AP.
:since: 03/12/2012 RTC26478
:author: apairex
"""
from acs_test_scripts.UseCase.Networking.LAB_WIFI_DUAL_BASE import LabWifiDualBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.UECmdTypes import AUTO_CONNECT_STATE

from time import sleep
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiDualApBandSelConnect(LabWifiDualBase):

    """
    Lab Wifi Dual AP Frequency Band Selection Connect class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiDualBase.__init__(self, tc_name, global_config)

        self._24to5 = str(self._tc_parameters.get_param_value("2_4_TO_5_GHZ"))
        self._5to24 = str(self._tc_parameters.get_param_value("5_TO_2_4_GHZ"))
        if self._24to5.lower() in ["true", "1", "yes"]:
            self._24to5 = True
        else:
            self._24to5 = False
        if self._5to24.lower() in ["true", "1", "yes"]:
            self._5to24 = True
        else:
            self._5to24 = False

        self._test_sequence = []

        # AP must not be hidden
        self._hidden = False
        self._hidden_ap2 = False

        self._reconnection_time = 10

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # pylint: disable=E1101
        LabWifiDualBase.set_up(self)

        # Control that the TC parameters has been correctly set.
        if self._standard not in ["a", "n5G"]:
            msg = "AP1 is not set to 5GHz. Standard set: %s" % self._standard
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._standard_ap2 not in ["gb", "ngb", "n2.4G"]:
            msg = "AP2 is not set to 2.4GHz. Standard set: %s" \
                % self._standard_ap2
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._channel != "1":
            self._logger.warning("AP1 channel is not 1 as advise: %s"
                                 % self._channel)
        if self._channel_ap2 != "1":
            self._logger.warning("AP2 channel is not 1 as advise: %s"
                                 % self._channel_ap2)

        if not self._24to5 and not self._5to24:
            msg = "At least 1 of the parameter 2_4_TO_5_GHZ or 5_TO_2_4_GHZ "
            msg += "should be enabled"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Set band selection to "auto"
        self._networking_api.set_wifi_frequency_band("auto", False, self._dut_wlan_iface)

        if self._24to5:
            # Connect to 2.4GHz AP
            self._networking_api.wifi_connect(self._ssid_ap2)
            self._test_sequence += [["5GHz", self._ssid]]
        if self._5to24:
            if not self._24to5:
                # Connect to 5GHz AP
                self._networking_api.wifi_connect(self._ssid)
            self._test_sequence += [["2.4GHz", self._ssid_ap2]]

        # Set autoconnect for both SSIDs
        self._networking_api.set_autoconnect_mode(self._ssid,
                                                  AUTO_CONNECT_STATE.on)
        self._networking_api.set_autoconnect_mode(self._ssid_ap2,
                                                  AUTO_CONNECT_STATE.on)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiDualBase.run_test(self)

        for (cur_band, ssid) in self._test_sequence:
            # Set the new band
            self._networking_api.set_wifi_frequency_band(cur_band, False, self._dut_wlan_iface)
            sleep(self._wait_btwn_cmd)

            # Check the new band selection
            res_band = self._networking_api.get_wifi_frequency_band(self._dut_wlan_iface)

            # Compare it to what has been set
            if res_band != cur_band:
                msg = "Got band %s instead of %s" % (res_band, cur_band)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # Check auto re-connection the other ssid
            self._logger.info("Waiting % sec to reconnect"
                              % self._reconnection_time)
            sleep(self._reconnection_time)
            self._networking_api.check_connection_state(ssid)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiDualBase.tear_down(self)

        # Reset the Frequency Band parameter to "auto"
        self._networking_api.set_wifi_frequency_band("auto", False, self._dut_wlan_iface)

        return Global.SUCCESS, "No errors"
