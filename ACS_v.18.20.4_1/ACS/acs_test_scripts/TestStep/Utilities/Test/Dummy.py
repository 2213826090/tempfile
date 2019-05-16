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
:summary: This file implements a Test Step to validate Test Step mechanism
:since:03/12/2013
:author: cbonnard
"""

from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class Dummy(TestStepBase):

    """
        Validate Test Step mechanism
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        TestStepBase.run(self, context)

        self._logger.info("Saves input value (%s) into context using variable name (%s)...",
                          self._pars.input_1, self._pars.ctx_data_1)
        context.set_info(self._pars.ctx_data_1, self._pars.input_1)

        self._logger.info("Saves input values (%s and %s) into context bundle using variable name (%s)...",
                          self._pars.input_1, self._pars.input_2, self._pars.ctx_data_2)
        context.set_nested_info([self._pars.ctx_data_2, "INPUT_1"], self._pars.input_1)
        context.set_nested_info([self._pars.ctx_data_2, "INPUT_2"], self._pars.input_2)

        self._logger.info("Generates return code (%s), with comment (%s)...", self._pars.return_code,
                          self._pars.comment)
        self.ts_verdict_msg = self._pars.comment

        if self._pars.return_code == "SUCCESS":
            pass
        elif self._pars.return_code == "FAILURE":
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.comment)
        elif self._pars.return_code == "BLOCKED":
            raise AcsToolException(AcsToolException.OPERATION_FAILED, self._pars.comment)
        elif self._pars.return_code == "ACS_EXCEPTION":
            raise AcsToolException(AcsToolException.OPERATION_FAILED, self._pars.comment)
        elif self._pars.return_code == "DEVICE_EXCEPTION":
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.comment)
        elif self._pars.return_code == "UNKNOWN_EXCEPTION":
            raise ValueError(self._pars.comment)
        else:
            raise AcsToolException(AcsToolException.OPERATION_FAILED, self._pars.comment)
