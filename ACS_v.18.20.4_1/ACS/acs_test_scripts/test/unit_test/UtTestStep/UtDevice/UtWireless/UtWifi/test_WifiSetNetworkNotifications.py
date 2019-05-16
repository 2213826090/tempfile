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
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiSetNetworkNotifications import WifiSetNetworkNotifications
from Core.TestStep.TestStepContext import TestStepContext


class WifiSetNetworkNotificationsTest(UTestTestStepBase):
    """
    WifiSetNetworkNotifications test cases
    """

    WIFI_NETWORK_NOTIFICATIONS_ENABLED = 1
    WIFI_NETWORK_NOTIFICATIONS_DISABLED = 0
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

    def test_set_network_notifications_enabled_ok(self):
        sut = self._create_sut({"NETWORK_NOTIFICATIONS": True})
        self._simulate_get_wifi_network_notifications_returning(sut, [self.WIFI_NETWORK_NOTIFICATIONS_ENABLED])
        self._assert_run_returned(sut, self.WIFI_NETWORK_NOTIFICATIONS_ENABLED)

    def test_set_network_notifications_disabled_ok(self):
        sut = self._create_sut({"NETWORK_NOTIFICATIONS": False})
        self._simulate_get_wifi_network_notifications_returning(sut, [self.WIFI_NETWORK_NOTIFICATIONS_DISABLED])
        self._assert_run_returned(sut, self.WIFI_NETWORK_NOTIFICATIONS_DISABLED)

    def test_set_network_notifications_fail(self):
        sut = self._create_sut({"NETWORK_NOTIFICATIONS": True})
        self._simulate_get_wifi_network_notifications_returning(sut, [self.WIFI_NETWORK_NOTIFICATIONS_DISABLED])
        self._assert_run_throw_device_exception(sut, "Set WiFi network notifications True failure")

    def test_set_network_notifications_wifi_off_fail(self):
        self._wifi_power_status = self.WIFI_OFF
        sut = self._create_sut({"NETWORK_NOTIFICATIONS": True})
        self._simulate_get_wifi_network_notifications_returning(sut, [self.WIFI_NETWORK_NOTIFICATIONS_DISABLED])
        self._assert_run_throw_device_exception(sut, "Can't set WiFi network notifications, WiFi is currently OFF")

    def test_set_network_notifications_ok_after_several_tries(self):
        sut = self._create_sut({"NETWORK_NOTIFICATIONS": True})
        sut._timeout = 5
        self._simulate_get_wifi_network_notifications_returning(sut, [self.WIFI_NETWORK_NOTIFICATIONS_DISABLED,
                                                               self.WIFI_NETWORK_NOTIFICATIONS_DISABLED,
                                                               self.WIFI_NETWORK_NOTIFICATIONS_ENABLED])
        self._assert_run_returned(sut, self.WIFI_NETWORK_NOTIFICATIONS_ENABLED)

    # ------------------------------------------------------------------------------------------------------------------

    def _side_effect_get_network_notification_response(self):
        return self._responsed.pop(0)

    def _simulate_get_wifi_network_notifications_returning(self, sut, expected_state):
        self._responsed = expected_state
        sut._api.get_network_notification.side_effect = self._side_effect_get_network_notification_response
        return sut

    def _assert_run_returned(self, sut, state):
        sut.run(None)
        sut._api.set_network_notification.assert_called_once_with(state)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiSetNetworkNotifications(None, None, test_step_pars, mock.Mock())
        sut._api.get_wifi_power_status = self._return_wifi_power_status
        sut._timeout = 0
        return sut

