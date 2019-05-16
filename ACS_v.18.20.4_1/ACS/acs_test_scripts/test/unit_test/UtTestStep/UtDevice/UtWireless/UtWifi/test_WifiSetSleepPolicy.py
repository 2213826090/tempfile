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
:since 07/08/2014
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiSetSleepPolicy import WifiSetSleepPolicy
from Core.TestStep.TestStepContext import TestStepContext


class WifiSetSleepPolicyTest(UTestTestStepBase):
    """
    WifiSetSleepPolicy test cases
    """

    WIFI_POLICY_ALWAYS = "ALWAYS"
    WIFI_POLICY_WHEN_PLUGGED = "ONLY_WHEN_PLUGGED"
    WIFI_POLICY_NEVER = "NEVER"
    WIFI_POLICY_ALWAYS_VALUE = 0x2
    WIFI_POLICY_WHEN_PLUGGED_VALUE = 0x1
    WIFI_POLICY_NEVER_VALUE = 0x0
    WIFI_ON = 1
    WIFI_OFF = 0

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._set_timeout = 0
        self._wifi_power_status = self.WIFI_ON
        self._responsed = []

    def _return_wifi_power_status(self):
        """
        Stub method
        """
        return self._wifi_power_status

    def test_set_wifi_sleep_policy_always_ok(self):
        sut = self._create_sut({"SLEEP_POLICY": self.WIFI_POLICY_ALWAYS})
        self._simulate_get_wifi_sleep_policy_returning(sut, [self.WIFI_POLICY_ALWAYS_VALUE])
        self._assert_run_returned(sut, self.WIFI_POLICY_ALWAYS_VALUE)

    def test_set_wifi_sleep_policy_never_ok(self):
        sut = self._create_sut({"SLEEP_POLICY": self.WIFI_POLICY_NEVER})
        self._simulate_get_wifi_sleep_policy_returning(sut, [self.WIFI_POLICY_NEVER_VALUE])
        self._assert_run_returned(sut, self.WIFI_POLICY_NEVER_VALUE)

    def test_set_wifi_sleep_policy_only_when_plugged_ok(self):
        sut = self._create_sut({"SLEEP_POLICY": self.WIFI_POLICY_WHEN_PLUGGED})
        self._simulate_get_wifi_sleep_policy_returning(sut, [self.WIFI_POLICY_WHEN_PLUGGED_VALUE])
        self._assert_run_returned(sut, self.WIFI_POLICY_WHEN_PLUGGED_VALUE)

    def test_set_wifi_sleep_policy_fail(self):
        sut = self._create_sut({"SLEEP_POLICY": self.WIFI_POLICY_ALWAYS})
        self._simulate_get_wifi_sleep_policy_returning(sut, [self.WIFI_POLICY_WHEN_PLUGGED_VALUE])
        self._assert_run_throw_device_exception(sut, "Set WiFi sleep policy %s failure" % self.WIFI_POLICY_ALWAYS)

    def test_set_wifi_sleep_policy_wifi_off_fail(self):
        self._wifi_power_status = self.WIFI_OFF
        sut = self._create_sut({"SLEEP_POLICY": self.WIFI_POLICY_ALWAYS})
        self._simulate_get_wifi_sleep_policy_returning(sut, [self.WIFI_POLICY_ALWAYS_VALUE])
        self._assert_run_throw_device_exception(sut, "Can't set WiFi sleep policy, WiFi is currently OFF")

    def test_set_network_notifications_ok_after_several_tries(self):
        sut = self._create_sut({"SLEEP_POLICY": self.WIFI_POLICY_ALWAYS})
        sut._timeout = 5
        self._simulate_get_wifi_sleep_policy_returning(sut, [self.WIFI_POLICY_WHEN_PLUGGED_VALUE,
                                                               self.WIFI_POLICY_WHEN_PLUGGED_VALUE,
                                                               self.WIFI_POLICY_ALWAYS_VALUE])
        self._assert_run_returned(sut, self.WIFI_POLICY_ALWAYS_VALUE)

    # ------------------------------------------------------------------------------------------------------------------

    def _side_effect_get_wifi_sleep_policy_response(self):
        return self._responsed.pop(0)

    def _simulate_get_wifi_sleep_policy_returning(self, sut, expected_state):
        self._responsed = expected_state
        sut._api.get_wifi_sleep_policy.side_effect = self._side_effect_get_wifi_sleep_policy_response
        return sut

    def _assert_run_returned(self, sut, state):
        sut.run(None)
        sut._api.set_wifi_sleep_policy.assert_called_once_with(state)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiSetSleepPolicy(None, None, test_step_pars, mock.Mock())
        sut._api.get_wifi_power_status = self._return_wifi_power_status
        sut._timeout = 0
        return sut

