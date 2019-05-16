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
:summary: This file implements a Test Step to suspend execution
:since:15/03/2013
:author: fbongiax
"""

import time
from Core.TestStep.TestStepBase import TestStepBase


class SuspendToTime(TestStepBase):
    """
    Suspend the execution until specified time
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        wait_time = self._pars.to_time - time.time()
        if wait_time < 0:
            self._logger.info("Reach timestamp since {0} seconds".format(abs(wait_time)))
            return
        else:
            self._logger.info("Suspend execution for {0} seconds".format(wait_time))
            time.sleep(wait_time)
