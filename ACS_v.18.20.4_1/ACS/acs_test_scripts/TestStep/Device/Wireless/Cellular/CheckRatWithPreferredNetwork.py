"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: This file implements a Test Step to check RAT with preferred network
:since: 19/09/2014
:author: jfranchx
"""

import time
from acs_test_scripts.TestStep.Device.Wireless.Cellular.CellularBase import CellularBase
from acs_test_scripts.Device.UECmd.UECmdTypes import NETWORK_PREFERENCES


class CheckRatWithPreferredNetwork(CellularBase):
    """
    Implements the test step to check RAT with preferred network
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        CellularBase.run(self, context)
        assert self._pars.preferred_network in NETWORK_PREFERENCES, "passed value (%s) is invalid at this stage" % self._pars.preferred_network

        # Configure optional parameters
        if self._pars.loop_timer is not None:
            if self._pars.loop_timer < 0:
                loop_timer = 0
                self._logger.warning("Invalid LOOP_TIMER value, use 0 as default value")
            else:
                loop_timer = int(self._pars.loop_timer)
        else:
            loop_timer = 0
        if self._pars.timeout_between_checks is not None:
            if self._pars.timeout_between_checks < 0:
                timeout_between_checks = 0
                self._logger.warning("Invalid TIMEOUT_BETWEEN_CHECKS value, use 0 as default value")
            else:
                timeout_between_checks = int(self._pars.timeout_between_checks)
        else:
            timeout_between_checks = 0

        initial_time = time.time()
        first_loop = False

        while loop_timer > (time.time() - initial_time) or first_loop is False:
            self._modem_api.check_rat_with_pref_network(self._pars.preferred_network, int(self._pars.timeout))
            time.sleep(timeout_between_checks)
            first_loop = True
            my_time = time.time()
