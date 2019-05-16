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
:summary: This file implements a Test Step to get a device BT Power Status
:since: 17/04/2014
:author: jfranchx
"""
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase

from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE

from UtilitiesFWK.Utilities import TestConst

class BtGetPower(BtBase):
    """
    Implements the test step to get BT Power Status
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        # Get the BT power status and post it in the context
        power_status = self._api.get_bt_power_status()
        if power_status == str(BT_STATE.STATE_ON):
            power_status = TestConst.STR_ON
        if power_status == str(BT_STATE.STATE_OFF):
            power_status = TestConst.STR_OFF

        context.set_info(self._pars.save_as, power_status)

        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(power_status) % self._pars.save_as
        self._logger.debug(self.ts_verdict_msg)
