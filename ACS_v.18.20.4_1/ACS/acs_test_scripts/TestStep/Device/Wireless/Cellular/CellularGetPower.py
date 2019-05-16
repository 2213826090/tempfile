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
:summary: This file implements a Test Step to get Cellular power status
:since 30/10/2014
:author: emarchan
"""
from acs_test_scripts.TestStep.Device.Wireless.Cellular.CellularBase import CellularBase

from UtilitiesFWK.Utilities import TestConst, str_to_bool_ex


class CellularGetPower(CellularBase):
    """
    Implements the Power test step for Cellular
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        CellularBase.run(self, context)

        self._logger.info("Getting the Cellular interface state...")

        # Where the information will be stored into the context?
        variable_to_set = self._pars.save_as

        value = self._modem_api.get_modem_power_status()
        if str_to_bool_ex(value):
            value = TestConst.STR_ON
        else:
            value = TestConst.STR_OFF

        # We have the value, let's save it into the context
        context.set_info(variable_to_set, value)
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(value) % variable_to_set
        self._logger.debug(self.ts_verdict_msg)
