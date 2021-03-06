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

:organization: INTEL NDG
:summary: This file implements the split function of string
:since: 13/01/2015
:author: gcharlex
"""
from Core.TestStep.TestStepBase import TestStepBase


class Split(TestStepBase):
    """
    AT command class
    """
    def __init__(self, tc_name, global_config, ts_conf, factory):
        """
        Constructor
        """
        # Call DeviceTestStepBase base Init function
        TestStepBase.__init__(self, tc_name, global_config, ts_conf, factory)


    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        result = self._pars.string.split(self._pars.sep)
        if self._pars.element is not None:
            result = result[self._pars.element]

        # Save the command result in the context variable
        context.set_info(self._pars.save_as, result)
