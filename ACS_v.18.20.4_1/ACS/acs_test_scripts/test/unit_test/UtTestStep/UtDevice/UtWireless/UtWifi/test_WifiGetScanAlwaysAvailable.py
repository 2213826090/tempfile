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
:since 06/01/2015
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiGetScanAlwaysAvailable import WifiGetScanAlwaysAvailable


class WifiGetScanAlwaysAvailableTest(UTestTestStepBase):
    """
    WifiGetScanAlwaysAvailable test cases
    """

    WIFI_SCAN_ALWAYS_AVAILABLE_ENABLED = "ON"
    WIFI_SCAN_ALWAYS_AVAILABLE_DISABLED = "OFF"
    GOT_SCAN_ALWAYS_AVAILABLE_ENABLED = "ON"
    GOT_SCAN_ALWAYS_AVAILABLE_DISABLED = "OFF"
    MSG_INVALID_VALUE = "ERROR - Unknown WiFi scan always available state : INVALID_VALUE"
    CONTEXT_SAVE_GET = "DUT_SCAN_ALWAYS_AVAILABLE"

    def _return_wifi_scan_always_available(self):
        """
        Stub method
        """
        return self._scan_always_available

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._scan_always_available = None
        self._context = TestStepContext()

    def test_get_scan_always_available_enable_ok(self):
        self._scan_always_available = self.WIFI_SCAN_ALWAYS_AVAILABLE_ENABLED
        sut = self._create_sut({"SAVE_SCAN_ALWAYS_AVAILABLE": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(
            self.GOT_SCAN_ALWAYS_AVAILABLE_ENABLED) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.GOT_SCAN_ALWAYS_AVAILABLE_ENABLED, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_scan_always_available_disable_ok(self):
        self._scan_always_available = self.WIFI_SCAN_ALWAYS_AVAILABLE_DISABLED
        sut = self._create_sut({"SAVE_SCAN_ALWAYS_AVAILABLE": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(
            self.GOT_SCAN_ALWAYS_AVAILABLE_DISABLED) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.GOT_SCAN_ALWAYS_AVAILABLE_DISABLED, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_scan_always_available_fail(self):
        self._scan_always_available = "INVALID_VALUE"
        sut = self._create_sut({"SAVE_SCAN_ALWAYS_AVAILABLE": self.CONTEXT_SAVE_GET})
        self._assert_run_throw_device_exception(sut, self.MSG_INVALID_VALUE)

    # pylint: disable=W0212
    def _create_sut(self, args=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiGetScanAlwaysAvailable(None, None, args, mock.Mock())
        sut._api.get_wifi_scan_always_available = self._return_wifi_scan_always_available
        return sut
