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
:summary: This file implements a Test Step to set NFC power
:since 05/09/2014
:author: jfranchx
"""

import time

from UtilitiesFWK.Utilities import split_and_strip
from acs_test_scripts.TestStep.Device.Wireless.NFC.NFCBase import NFCBase


class NFCSetPower(NFCBase):
    """
    Implements the set power test step for NFC
    """

    # Constants
    STR_NFC_ON = "on"
    STR_NFC_OFF = "off"
    STR_SEPARATOR = ","
    SET_DEFAULT_TIMEOUT_SECS = 5

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        NFCBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._initial_time = None
        self._timeout = self.SET_DEFAULT_TIMEOUT_SECS
        self._wait_for = 1
        self._state_reached = False
        self._nfc_power = None

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        NFCBase.run(self, context)

        # Get the power sequence (it might just be one single operation)
        power = str(self._pars.power).lower()
        power_sequence = split_and_strip(power, self.STR_SEPARATOR)

        for current in power_sequence:
            assert current.lower() in [self.STR_NFC_ON, self.STR_NFC_OFF], \
            "passed value (%s) is invalid at this stage" % current

            self._logger.info("Powering %s the NFC interface" % current)

            if current == self.STR_NFC_ON:
                self._api.nfc_enable()
            else:
                self._api.nfc_disable()

            self._wait_until_state_is_reached(str(current).upper())

    def _wait_until_state_is_reached(self, expected):
        self._initial_time = time.time()
        while not(self._is_state_reached(expected) or self._time_out_reached()):
            time.sleep(self._wait_for)

        if self._time_out_reached() and not self._state_reached:
            self._raise_device_exception("Set NFC %s failure" % expected.lower())

    def _time_out_reached(self):
        return time.time() - self._initial_time >= self._timeout;

    def _is_state_reached(self, expected):
        # Get state
        state = self._api.get_nfc_status()
        self._state_reached = (expected == state)
        return self._state_reached
