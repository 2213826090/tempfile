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
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiSetScanAlwaysAvailable import WifiSetScanAlwaysAvailable
from Core.TestStep.TestStepContext import TestStepContext


class WifiSetScanAlwaysAvailableTest(UTestTestStepBase):
    """
    WifiSetScanAlwaysAvailable test cases
    """

    WIFI_SCAN_ALWAYS_AVAILABLE_ENABLED = "ON"
    WIFI_SCAN_ALWAYS_AVAILABLE_DISABLED = "OFF"

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._set_timeout = 0
        self._responsed = []

    def test_set_scan_always_available_enabled_ok(self):
        sut = self._create_sut({"SCAN_ALWAYS_AVAILABLE_STATE": "ON"})
        self._simulate_get_wifi_scan_always_available_returning(sut, [self.WIFI_SCAN_ALWAYS_AVAILABLE_ENABLED])
        self._assert_run_returned(sut, self.WIFI_SCAN_ALWAYS_AVAILABLE_ENABLED)

    def test_set_scan_always_available_disabled_ok(self):
        sut = self._create_sut({"SCAN_ALWAYS_AVAILABLE_STATE": "OFF"})
        self._simulate_get_wifi_scan_always_available_returning(sut, [self.WIFI_SCAN_ALWAYS_AVAILABLE_DISABLED])
        self._assert_run_returned(sut, self.WIFI_SCAN_ALWAYS_AVAILABLE_DISABLED)

    def test_set_scan_always_available_fail(self):
        sut = self._create_sut({"SCAN_ALWAYS_AVAILABLE_STATE": "ON"})
        self._simulate_get_wifi_scan_always_available_returning(sut, [self.WIFI_SCAN_ALWAYS_AVAILABLE_DISABLED])
        self._assert_run_throw_device_exception(sut, "Set WiFi scan always available option ON failure")

    def test_set_scan_always_available_ok_after_several_tries(self):
        sut = self._create_sut({"SCAN_ALWAYS_AVAILABLE_STATE": "ON"})
        sut._timeout = 5
        self._simulate_get_wifi_scan_always_available_returning(sut, [self.WIFI_SCAN_ALWAYS_AVAILABLE_DISABLED,
                                                               self.WIFI_SCAN_ALWAYS_AVAILABLE_DISABLED,
                                                               self.WIFI_SCAN_ALWAYS_AVAILABLE_ENABLED])
        self._assert_run_returned(sut, self.WIFI_SCAN_ALWAYS_AVAILABLE_ENABLED)

    # ------------------------------------------------------------------------------------------------------------------

    def _side_effect_get_scan_always_available_response(self):
        return self._responsed.pop(0)

    def _simulate_get_wifi_scan_always_available_returning(self, sut, expected_state):
        self._responsed = expected_state
        sut._api.get_wifi_scan_always_available.side_effect = self._side_effect_get_scan_always_available_response
        return sut

    def _assert_run_returned(self, sut, state):
        sut.run(None)
        sut._api.set_wifi_scan_always_available.assert_called_once_with(state)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiSetScanAlwaysAvailable(None, None, test_step_pars, mock.Mock())
        sut._timeout = 0
        return sut

