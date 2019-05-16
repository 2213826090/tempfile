"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW
:summary: Implements a test step that checks device discoverable timeout value
:since:19/09/2014
:author: ymorelx
"""
from TestStep.Device.Wireless.BT.Base import BtBase


class BtCheckDiscoverableTimeout(BtBase):
    """
    Implements the test step to check BT device discoverable timeout value
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        expected_timeout = self._pars.timeout
        # Get the BT current discoverable timeout
        discoverable_timeout = self._api.get_bt_discoverable_timeout()

        if discoverable_timeout != expected_timeout:
            self._raise_device_exception(
                "Expected %d discoverable timeout, got %d" %
                    (discoverable_timeout, expected_timeout)
            )
