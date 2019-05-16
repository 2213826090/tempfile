"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:since 06/08/2014
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiSetFrequencyBand import WifiSetFrequencyBand
from Core.TestStep.TestStepContext import TestStepContext


class WifiSetFrequencyBandTest(UTestTestStepBase):
    """
    WifiSetFrequencyBand test cases
    """

    FREQUENCY_BAND_AUTO = "auto"
    FREQUENCY_BAND_2_4GHZ = "2.4GHz"
    FREQUENCY_BAND_5GHZ = "5GHz"
    WIFI_INTERFACE = "wlan0"
    WIFI_ON = 1
    WIFI_OFF = 0
    SILENT_MODE_ON = True
    SILENT_MODE_OFF = False

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._wifi_power_status = self.WIFI_ON
        self._responsed = []

    def _return_wifi_power_status(self):
        """
        Stub method
        """
        return self._wifi_power_status

    def test_set_frequency_band_2_4ghz_ok(self):
        sut = self._create_sut({"FREQUENCY": "2_4GHZ", "INTERFACE": self.WIFI_INTERFACE})
        self._simulate_get_wifi_frequency_band_returning(sut, [self.FREQUENCY_BAND_2_4GHZ])
        self._assert_run_returned(sut, self.FREQUENCY_BAND_2_4GHZ, self.WIFI_INTERFACE, self.SILENT_MODE_OFF)

    def test_set_frequency_band_5ghz_ok(self):
        sut = self._create_sut({"FREQUENCY": "5GHZ", "INTERFACE": self.WIFI_INTERFACE})
        self._simulate_get_wifi_frequency_band_returning(sut, [self.FREQUENCY_BAND_5GHZ])
        self._assert_run_returned(sut, self.FREQUENCY_BAND_5GHZ, self.WIFI_INTERFACE, self.SILENT_MODE_OFF)

    def test_set_frequency_band_auto_ok(self):
        sut = self._create_sut({"FREQUENCY": "AUTO", "INTERFACE": self.WIFI_INTERFACE})
        self._simulate_get_wifi_frequency_band_returning(sut, [self.FREQUENCY_BAND_AUTO])
        self._assert_run_returned(sut, self.FREQUENCY_BAND_AUTO, self.WIFI_INTERFACE, self.SILENT_MODE_OFF)

    def test_set_frequency_band_fail(self):
        sut = self._create_sut({"FREQUENCY": "2_4GHZ", "INTERFACE": self.WIFI_INTERFACE})
        self._simulate_get_wifi_frequency_band_returning(sut, [self.FREQUENCY_BAND_AUTO])
        self._assert_run_throw_device_exception(sut, "Set WiFi frequency band 2_4GHZ failure")

    def test_set_frequency_band_wifi_off_fail(self):
        self._wifi_power_status = self.WIFI_OFF
        sut = self._create_sut({"FREQUENCY": "2_4GHZ", "INTERFACE": self.WIFI_INTERFACE})
        self._simulate_get_wifi_frequency_band_returning(sut, [self.FREQUENCY_BAND_AUTO])
        self._assert_run_throw_device_exception(sut, "Can't set WiFi frequency band, WiFi is currently OFF")

    def test_set_frequency_band_ok_after_several_tries(self):
        sut = self._create_sut({"FREQUENCY": "AUTO", "INTERFACE": self.WIFI_INTERFACE})
        sut._timeout = 5
        self._simulate_get_wifi_frequency_band_returning(sut, [self.FREQUENCY_BAND_2_4GHZ, self.FREQUENCY_BAND_2_4GHZ,
                                                               self.FREQUENCY_BAND_AUTO])
        self._assert_run_returned(sut, self.FREQUENCY_BAND_AUTO, self.WIFI_INTERFACE, self.SILENT_MODE_OFF)

    def test_set_frequency_band_silent_mode_on_ok(self):
        sut = self._create_sut({"FREQUENCY": "2_4GHZ", "INTERFACE": self.WIFI_INTERFACE, "SILENT_MODE": self.SILENT_MODE_ON})
        self._simulate_get_wifi_frequency_band_returning(sut, [self.FREQUENCY_BAND_5GHZ])
        self._assert_run_returned(sut, self.FREQUENCY_BAND_2_4GHZ, self.WIFI_INTERFACE, self.SILENT_MODE_ON)

    # ------------------------------------------------------------------------------------------------------------------

    def _side_effect_get_wifi_frequency_band_response(self, silent_mode):
        return self._responsed.pop(0)

    def _simulate_get_wifi_frequency_band_returning(self, sut, expected_state):
        self._responsed = expected_state
        sut._api.get_wifi_frequency_band.side_effect = self._side_effect_get_wifi_frequency_band_response
        return sut

    def _assert_run_returned(self, sut, frequency, interface, silent_mode):
        sut.run(None)
        sut._api.set_wifi_frequency_band.assert_called_once_with(frequency, interface=interface, silent_mode=silent_mode)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiSetFrequencyBand(None, None, test_step_pars, mock.Mock())
        sut._api.get_wifi_power_status = self._return_wifi_power_status
        sut._timeout = 0
        return sut

