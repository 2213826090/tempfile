"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.


:organization: INTEL MCG PSI
:summary: This file implements a Test Step to enable or disable the cellular coex manager traces.
:since 07/11/2014
:author: emarchan
"""

from acs_test_scripts.TestStep.Device.Wireless.Cellular.CellularBase import CellularBase
from UtilitiesFWK.Utilities import TestConst
from time import sleep


class SetCoexManagerTraces(CellularBase):
    """
    Implements the step to enable or disable the cellular coex manager traces.
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        CellularBase.run(self, context)

        state = str(self._pars.coex_manager_traces_state).lower()

        assert state in [TestConst.STR_ON, TestConst.STR_OFF], \
            "passed value (%s) is invalid at this stage" % state

        mode = ""
        if state == TestConst.STR_ON:
            mode = "VERBOSE"
        cur_state = self._modem_api.get_lte_coex_manager_messages_state()
        if cur_state != state:
            self._logger.info("Setting the Coex manager traces %s" % state)
            self._device.disconnect_board()
            self._modem_api.set_lte_coex_manager_messages(mode)
            self._device.connect_board()
        else:
            self._logger.info("The Coex manager traces are already %s " % state)
