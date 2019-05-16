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
:summary: This file implements a Test Step to set a BT device discoverable
:since:18/12/2013
:author: fbongiax
"""

from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants


class BtSetDiscoverable(BtBase):
    """
    Implements the test step for a BT device to be set discoverable
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        if self._pars.mode is None:
            self._pars.mode = Constants.DISCOVERABLE_BOTH

        if self._pars.timeout is None:
            self._pars.timeout = 0

        # # Does it make sense to have mode default = both?
        self._api.set_bt_discoverable(self._pars.mode, int(self._pars.timeout))
