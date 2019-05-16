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
:summary: unit test
:since: 28/07/2014
:author: jfranchx
"""
import unittest2 as unittest
import mock
import time

from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiSetHotspot import WifiSetHotspot
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class WifiSetHotspotTest(UTestTestStepBase):
    """
    WifiSetHotspot test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._flight_mode = 0
        self._responsed = []

    def _return_get_flight_mode(self):
        return self._flight_mode

    def test_set_power_and_configure_open_ok(self):
        """
        Test set power on and configure open
        """
        sut = self._create_sut(
            {"POWER": "on", "SSID": "SSID", "SECURITY": "OPEN", "STANDARD": "5GHZ_20MHZ", "CHANNEL": "36",
             "HIDDEN": "off"})
        self._simulate_get_wifi_hotspot_status_returning(sut,["on"])
        self._assert_run_returned(sut, "on", "SSID", "OPEN", None, "5GHZ_20MHZ", "36", 'off')

    def test_set_power_and_configure_wpa2_ok(self):
        """
        Test set power on and configure wpa2
        """
        sut = self._create_sut(
            {"POWER": "on", "SSID": "SSID", "SECURITY": "WPA2", "PASSPHRASE": "123456789", "STANDARD": "5GHZ_40MHZ",
             "CHANNEL": "40", "HIDDEN": "off"})
        self._simulate_get_wifi_hotspot_status_returning(sut,[1])
        self._assert_run_returned(sut, "on", "SSID", "WPA2", "123456789", "5GHZ_40MHZ", "40", 'off')

    def test_set_power_off_ok(self):
        """
        Test set power off
        """
        sut = self._create_sut({"POWER": "off"})
        self._simulate_get_wifi_hotspot_status_returning(sut,[0])
        self._assert_run_returned(sut, "off")

    def test_set_power_fail(self):
        """
        Test set power fail
        """
        sut = self._create_sut({"POWER": "on"})
        self._simulate_get_wifi_hotspot_status_returning(sut,[0])
        self._assert_run_throw_device_exception(sut, "Set WIFI HOTSPOT on failure")

    def test_set_power_flight_mode_enabled(self):
        """
        Test set power on with flight mode
        """
        sut = self._create_sut({"POWER": "on"})
        self._flight_mode = 1
        self._simulate_get_wifi_hotspot_status_returning(sut,[0])
        self._assert_run_throw_device_exception(sut, "Flight mode enabled ! Enabling Hotspot is not possible")

    def test_power_on_ok_after_several_tries(self):
        """
        Test set power on success after several tries
        """
        sut = self._create_sut({"POWER": "on"})
        sut._timeout = 5
        self._simulate_get_wifi_hotspot_status_returning(sut,[0, 0, 0, 1])
        self._assert_run_returned(sut, "on")

    # ------------------------------------------------------------------------------------------------------------------

    def _side_effect_get_wifi_hotspot_status_response(self):
        return self._responsed.pop(0)

    def _simulate_get_wifi_hotspot_status_returning(self, sut, expected_power):
        self._responsed = expected_power
        sut._api.get_wifi_hotspot_status.side_effect = self._side_effect_get_wifi_hotspot_status_response
        return sut

    def _assert_run_returned(self, sut, state, ssid=None, security=None, passphrase=None, standard=None, channel=None, hidden="none"):
        sut.run(None)
        sut._api.set_wifi_hotspot.assert_called_once_with(state, ssid, security, passphrase, standard, channel, hidden)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiSetHotspot(None, None, test_step_pars, mock.Mock())
        sut._api.get_flight_mode = self._return_get_flight_mode
        sut._timeout = 0
        return sut


if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
