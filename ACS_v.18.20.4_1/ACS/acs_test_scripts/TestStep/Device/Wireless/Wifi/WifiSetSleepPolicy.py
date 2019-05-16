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
:summary: This file implements a Test Step to set sleep policy
:since 03/09/2014
:author: jfranchx
"""

import time

from acs_test_scripts.TestStep.Device.Wireless.Wifi.Constants import Constants
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiBase import WifiBase
from ErrorHandling.DeviceException import DeviceException


class WifiSetSleepPolicy(WifiBase):
    """
    Implements the set sleep policy test step for Wifi
    """

    # Constants
    SET_DEFAULT_TIMEOUT_SECS = 5

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        WifiBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._initial_time = None
        self._timeout = self.SET_DEFAULT_TIMEOUT_SECS
        self._wait_for = 1
        self._state_reached = False
        self._sleep_policy = None

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiBase.run(self, context)
        assert self._pars.sleep_policy in ["ALWAYS", "ONLY_WHEN_PLUGGED", "NEVER"], \
            "passed value (%s) is invalid at this stage" % self._pars.sleep_policy

        self._logger.info("Set sleep policy for Wifi...")

        # Configure parameters with good values
        self._sleep_policy = Constants.WIFI_SLEEP_POLICY_ALL.get(self._pars.sleep_policy, "ERROR")

        # WiFi must be On to use this function
        if self._api.get_wifi_power_status() == 0:
            msg = "Can't set WiFi sleep policy, WiFi is currently OFF"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._api.set_wifi_sleep_policy(self._sleep_policy)
        self._wait_until_state_is_reached(self._sleep_policy)

    def _wait_until_state_is_reached(self, expected):
        self._initial_time = time.time()
        while not(self._is_state_reached(expected) or self._time_out_reached()):
            time.sleep(self._wait_for)

        if self._time_out_reached() and not self._state_reached:
            self._raise_device_exception("Set WiFi sleep policy %s failure" % self._pars.sleep_policy)

    def _time_out_reached(self):
        return time.time() - self._initial_time >= self._timeout;

    def _is_state_reached(self, expected):
        # Get state
        state = self._api.get_wifi_sleep_policy()
        self._state_reached = (expected == state)
        return self._state_reached
