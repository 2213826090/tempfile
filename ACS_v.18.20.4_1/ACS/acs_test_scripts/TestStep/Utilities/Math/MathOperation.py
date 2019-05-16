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
:summary: This file implements a Test Step to perform a simple operation
:since: 20/10/2014
:author: jfranchx
"""
from Core.TestStep.TestStepBase import TestStepBase
from UtilitiesFWK.Utilities import is_number
from ErrorHandling.AcsConfigException import AcsConfigException


class MathOperation (TestStepBase):
    """
    Mathematical operation
    """
    ADD = "ADD"
    SUBTRACT = "SUBTRACT"
    MULTIPLY = "MULTIPLY"
    DIVIDE = "DIVIDE"

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._result = None

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        TestStepBase.run(self, context)

        assert self._pars.operator in [self.ADD, self.SUBTRACT, self.MULTIPLY, self.DIVIDE], \
        "Operator value is invalid (it should have been checked by the framework)"

        first_value = float(self._pars.first)
        second_value = float(self._pars.second)

        if self._pars.operator == self.ADD:
            self._result = first_value + second_value
        elif self._pars.operator == self.SUBTRACT:
            self._result = first_value - second_value
        elif self._pars.operator == self.MULTIPLY:
            self._result = first_value * second_value
        elif self._pars.operator == self.DIVIDE:
            if second_value == 0:
                msg = "Second value = 0 ! Division by 0 is not possible"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            else:
                self._result = first_value / second_value

        context.set_info(self._pars.save_result_as, str(self._result))

        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(self._result) % self._pars.save_result_as
        self._logger.debug(self.ts_verdict_msg)
