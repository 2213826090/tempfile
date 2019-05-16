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
from ErrorHandling.AcsConfigException import AcsConfigException


class Suspend(TestStepBase):

    """
    Suspend the execution for N seconds
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)
        self._logger.info("Suspend execution for {0} seconds".format(self._pars.duration_in_sec))
        if self._pars.refresh_stdout_in_sec and self._pars.refresh_stdout_in_sec > self._pars.duration_in_sec:
            msg = "REFRESH_STDOUT_IN_SEC value is superior to DURATION_IN_SEC value"
            self._logger.info(msg)
            time.sleep(self._pars.duration_in_sec)
            return
        if not self._pars.refresh_stdout_in_sec:
            time.sleep(self._pars.duration_in_sec)
        else:
            div_duration_by_refresh, remainder = divmod(self._pars.duration_in_sec,
                                                        self._pars.refresh_stdout_in_sec)
            # duration_in_sec = refresh_stdout_in_sec * div_duration_by_refresh + remainder
            for i in range(1, int(div_duration_by_refresh) + 1):
                elapsed_time = i * self._pars.refresh_stdout_in_sec
                time.sleep(self._pars.refresh_stdout_in_sec)
                self._logger.info("Suspend execution since {0} seconds".format(elapsed_time))
            time.sleep(remainder)
