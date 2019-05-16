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
:summary: This file implements a Test Step to power Wifi
:since 26/03/2014
:author: emarchan
"""

import time

from UtilitiesFWK.Utilities import split_and_strip, TestConst, str_to_bool_ex
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiBase import WifiBase
from ErrorHandling.AcsConfigException import AcsConfigException


class WifiSetPower(WifiBase):
    """
    Implements the Power test step for Wifi
    """

    # Constants
    STR_SEPARATOR = ","
    POWER_DEFAULT_TIMEOUT_SECS = 5

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        WifiBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._initial_time = None
        self._timeout = self.POWER_DEFAULT_TIMEOUT_SECS
        self._wait_for = 1
        self._state_reached = False

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiBase.run(self, context)

        # get timeout if any
        timeout = self._pars.timeout
        if timeout:
            self._timeout = timeout
            self._logger.info("Timeout set to {0} for setting wifi power".format(timeout))

        # Get the power sequence (it might just be one single operation)
        power = str(self._pars.power).lower()
        power_sequence = split_and_strip(power, self.STR_SEPARATOR)

        for current in power_sequence:
            assert current.lower() in [TestConst.STR_ON, TestConst.STR_OFF], \
            "passed value (%s) is invalid at this stage" % current

            self._logger.info("Powering %s the Wifi interface" % current)

            self._api.set_wifi_power(current)

            self._wait_until_state_is_reached(current)

    def _wait_until_state_is_reached(self, expected):
        self._initial_time = time.time()
        while not(self._is_state_reached(expected) or self._time_out_reached()):
            time.sleep(self._wait_for)

        if self._time_out_reached() and not self._state_reached:
            self._raise_device_exception("Set WIFI %s failure" % expected)

    def _time_out_reached(self):
        return time.time() - self._initial_time >= self._timeout

    def _is_state_reached(self, expected):
        # Get state and convert it to on or off
        state = str_to_bool_ex(self._api.get_wifi_power_status())
        if state:
            state = TestConst.STR_ON
        else:
            state = TestConst.STR_OFF

        self._state_reached = (expected == state)
        return self._state_reached
