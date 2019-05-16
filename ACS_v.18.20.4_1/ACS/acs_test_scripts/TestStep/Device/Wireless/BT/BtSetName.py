"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file implements a Test Step to set a BT device name
@since 19/03/2014
@author: jfranchx
"""

from TestStep.Device.Wireless.BT.Base import BtBase


class BtSetName(BtBase):
    """
    Implements the test step for a BT device to set name
    Attributes:
        NAME: @see ILocalConnectivity.set_bt_name() name parameter
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        """
        Constructor
        """

        BtBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

#------------------------------------------------------------------------------

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        BtBase.run(self, context)

        self._api.set_bt_name(self._pars.name)
