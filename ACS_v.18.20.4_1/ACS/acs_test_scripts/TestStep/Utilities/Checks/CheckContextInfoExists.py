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
:summary: This file implements a Test Step to check if an info is present in the context
:since:15/03/2013
:author: fbongiax
"""
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException


class CheckContextInfoExists(TestStepBase):
    """
    Check that a key is present in the context
    """

    STR_KEY = "KEY"

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._key = self._pars.key

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        # Looks for the key in the context, raise an error if it doesn't exist
        info = context.get_info(self._key)
        if info is None:
            return_message = "%s is not found in the context" % self._key
            self._logger.error(return_message)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_message)
