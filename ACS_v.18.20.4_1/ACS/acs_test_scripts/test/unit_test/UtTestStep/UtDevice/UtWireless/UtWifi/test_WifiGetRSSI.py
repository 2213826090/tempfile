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
:since 04/11/2014
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiGetRSSI import WifiGetRSSI


class WifiGetRSSITest(UTestTestStepBase):
    """
    WifiGetRSSI test cases
    """

    RSSI_RETURN = -45
    SSID_TO_CHECK = "SSID_TO_CHECK"
    WIFI_ON = 1
    WIFI_OFF = 0
    CONTEXT_SAVE_GET = "DUT_WIFI_RSSI"

    def _return_wifi_rssi(self, ssid):
        """
        Stub method
        """
        return self._wifi_rssi

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
        self._wifi_rssi = self.RSSI_RETURN
        self._context = TestStepContext()
        self._wifi_power_status = self.WIFI_ON

    def test_get_wifi_rssi_ok(self):
        sut = self._create_sut({"SSID": self.SSID_TO_CHECK, "SAVE_WIFI_RSSI": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.RSSI_RETURN) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.RSSI_RETURN, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_wifi_rssi_wifi_off_fail(self):
        self._wifi_power_status = self.WIFI_OFF
        sut = self._create_sut({"SSID": self.SSID_TO_CHECK, "SAVE_WIFI_RSSI": self.CONTEXT_SAVE_GET})
        self._assert_run_throw_device_exception(sut, "Can't get WiFi RSSI, WiFi is currently OFF")

    # pylint: disable=W0212
    def _create_sut(self, args=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiGetRSSI(None, None, args, mock.Mock())
        sut._api.get_wifi_rssi = self._return_wifi_rssi
        sut._api.get_wifi_power_status = self._return_wifi_power_status
        return sut
