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
:summary: This file implements a Test Step to compare two values
:since:18/12/2013
:author: fbongiax
"""
from Core.TestStep.TestStepBase import TestStepBase
from UtilitiesFWK.Utilities import is_number
from ErrorHandling.AcsConfigException import AcsConfigException


class Compare(TestStepBase):
    """
    Check that two values satisfy the operation identified by the given operator
    """
    EQ = "EQUAL"
    NE = "NOT_EQUAL"
    GT = "GREATER"
    LT = "LESS"
    GE = "GREATER_OR_EQUAL"
    LE = "LESS_OR_EQUAL"
    IN = "IN"
    WITHIN_BOUNDS = "WITHIN_BOUNDS"

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        TestStepBase.run(self, context)

        assert self._pars.operator in [self.EQ, self.NE, self.GT, self.LT, self.GE, self.LE, self.IN, self.WITHIN_BOUNDS], \
        "Operator value is invalid (it should have been checked by the framework)"

        if self._pars.operator == self.IN:
            passed = self._pars.first in self._pars.second
        else:
            passed = self._compare_using_math_operators()

        passed = self._invert_passed_if_needed(passed)

        if not passed:
            self._raise_config_exception(AcsConfigException.OPERATION_FAILED, "Comparison is not satisfied")

    def _compare_using_math_operators(self):
        interval_radius = None
        if self._pars.operator == self.WITHIN_BOUNDS:
            temp = self._pars.second.split(',')
            if len(temp) == 2:
                self._pars.second = temp[0]
                interval_radius = temp[1]
            else:
                self._raise_config_exception("Must have exactly 2 elements in a comma separated list for SECOND",
                                             AcsConfigException.OPERATION_FAILED)
            if is_number(self._pars.first) and is_number(self._pars.second) and is_number(interval_radius):
                first = float(self._pars.first)
                second = float(self._pars.second)
                interval_radius = float(interval_radius)
            else:
                self._raise_config_exception("FIRST, SECOND and interval_radius(2nd element derived from separating SECOND) must be both numbers",
                                             AcsConfigException.OPERATION_FAILED)

            self._logger.info("Evaluate {0} {1} {2} with interval radius {3}; it's expected to be {4}".format(first,
                                                                                                              self._pars.operator,
                                                                                                              second,
                                                                                                              interval_radius,
                                                                                                              self._pars.pass_if))
        else:
            if is_number(self._pars.first) and is_number(self._pars.second):
                first = float(self._pars.first)
                second = float(self._pars.second)
            else:
                first = str(self._pars.first)
                second = str(self._pars.second)

            self._logger.info("Evaluate %s %s %s; it's expected to be %s" % (str(first), self._pars.operator, str(second),
                                                                             str(self._pars.pass_if)))

        return self._do_compare(first, second, self._pars.operator, interval_radius)

    def _invert_passed_if_needed(self, passed):
    # if pass_if == False it means the test passes if the condition is not verified
        if self._pars.pass_if == False:
            passed = not passed
        return passed

    def _do_compare(self, first, second, operator, interval_radius):
        """
        Execute comparison and returns True if it succeeds, or False otherwise
        """
        if operator == self.EQ:
            passed = first == second
        elif operator == self.NE:
            passed = first != second
        elif operator == self.LT:
            passed = first < second
        elif operator == self.GT:
            passed = first > second
        elif operator == self.LE:
            passed = first <= second
        elif operator == self.WITHIN_BOUNDS:
            passed = (second - interval_radius) <= first <= (second + interval_radius)
        else:
            assert operator == self.GE
            passed = first >= second

        return passed
