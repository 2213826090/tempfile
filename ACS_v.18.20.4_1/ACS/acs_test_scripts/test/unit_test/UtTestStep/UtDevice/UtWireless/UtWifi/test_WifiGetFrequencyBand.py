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
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiGetFrequencyBand import WifiGetFrequencyBand


class WifiGetFrequencyBandTest(UTestTestStepBase):
    """
    WiFiGetFrequencyBand test cases
    """

    FREQUENCY_BAND_AUTO = "auto"
    FREQUENCY_BAND_2_4GHZ = "2.4GHz"
    FREQUENCY_BAND_5GHZ = "5GHz"
    WIFI_INTERFACE = "wlan0"
    CONTEXT_SAVE_GET = "GET_SAVED"
    GOT_FREQUENCY_BAND_2_4GHZ = "2_4GHZ"
    GOT_FREQUENCY_BAND_5GHZ = "5GHZ"
    GOT_FREQUENCY_BAND_AUTO = "AUTO"
    MSG_INVALID_VALUE = "Error returned value get_wifi_frequency_band - INVALID_VALUE not supported"
    WIFI_ON = 1
    WIFI_OFF = 0

    def _return_wifi_frequency_band(self, silent_mode=False, interface="wlan0"):
        """
        Stub method
        """
        return self._frequency_band

    def _return_wifi_power_status(self):
        """
        Stub method
        """
        return self._wifi_power_status

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._frequency_band = self.FREQUENCY_BAND_AUTO
        self._context = TestStepContext()
        self._wifi_power_status = self.WIFI_ON

    def test_get_frequency_band_2_4ghz_ok(self):
        self._frequency_band = self.FREQUENCY_BAND_2_4GHZ
        sut = self._create_sut({"INTERFACE": "wlan0", "SAVE_FREQUENCY_BAND": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(
            self.GOT_FREQUENCY_BAND_2_4GHZ) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.GOT_FREQUENCY_BAND_2_4GHZ, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_frequency_band_5ghz_ok(self):
        self._frequency_band = self.FREQUENCY_BAND_5GHZ
        sut = self._create_sut({"INTERFACE": "wlan0", "SAVE_FREQUENCY_BAND": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(
            self.GOT_FREQUENCY_BAND_5GHZ) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.GOT_FREQUENCY_BAND_5GHZ, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_frequency_band_auto_ok(self):
        self._frequency_band = self.FREQUENCY_BAND_AUTO
        sut = self._create_sut({"INTERFACE": "wlan0", "SAVE_FREQUENCY_BAND": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(
            self.GOT_FREQUENCY_BAND_AUTO) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.GOT_FREQUENCY_BAND_AUTO, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_frequency_band_fail(self):
        self._frequency_band = "INVALID_VALUE"
        sut = self._create_sut({"INTERFACE": "wlan0", "SAVE_FREQUENCY_BAND": self.CONTEXT_SAVE_GET})
        self._assert_run_throw_device_exception(sut, self.MSG_INVALID_VALUE)

    def test_get_frequency_band_wifi_off_fail(self):
        self._frequency_band = self.FREQUENCY_BAND_5GHZ
        self._wifi_power_status = self.WIFI_OFF
        sut = self._create_sut({"INTERFACE": "wlan0", "SAVE_FREQUENCY_BAND": self.CONTEXT_SAVE_GET})
        self._assert_run_throw_device_exception(sut, "Can't get WiFi frequency band, WiFi is currently OFF")

    # pylint: disable=W0212
    def _create_sut(self, args=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiGetFrequencyBand(None, None, args, mock.Mock())
        sut._api.get_wifi_frequency_band = self._return_wifi_frequency_band
        sut._api.get_wifi_power_status = self._return_wifi_power_status
        return sut
