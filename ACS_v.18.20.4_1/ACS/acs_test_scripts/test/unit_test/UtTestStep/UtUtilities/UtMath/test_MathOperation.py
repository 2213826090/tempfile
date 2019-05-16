# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
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
:summary: Unit test module
:since: 20/10/14
:author: jfranchx
"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Utilities.Math.MathOperation import MathOperation
from Core.TestStep.TestStepContext import TestStepContext


class MathOperationTestCase(UTestTestStepBase):

    CTX_SAVE_RESULT = "RESULT_SAVED"

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._sut = None

    def test_add_positive_values(self):
        sut = self._create_sut("10", "12", MathOperation.ADD, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(22.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('22.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_add_negative_values(self):
        sut = self._create_sut("-10", "-12", MathOperation.ADD, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(-22.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('-22.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_add_positive_and_negative_values(self):
        sut = self._create_sut("10", "-12", MathOperation.ADD, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(-2.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('-2.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_add_negative_and_positive_values(self):
        sut = self._create_sut("-10", "12", MathOperation.ADD, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(2.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('2.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_subtract_positive_values(self):
        sut = self._create_sut("40", "12", MathOperation.SUBTRACT, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(28.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('28.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_subtract_negative_values(self):
        sut = self._create_sut("-10", "-22", MathOperation.SUBTRACT, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(12.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('12.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_subtract_positive_and_negative_values(self):
        sut = self._create_sut("10", "-12", MathOperation.SUBTRACT, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(22.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('22.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_subtract_negative_and_positive_values(self):
        sut = self._create_sut("-10", "12", MathOperation.SUBTRACT, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(-22.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('-22.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_multiply_positive_values(self):
        sut = self._create_sut("3", "3", MathOperation.MULTIPLY, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(9.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('9.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_multiply_negative_values(self):
        sut = self._create_sut("-3", "-3", MathOperation.MULTIPLY, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(9.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('9.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_multiply_positive_and_negative_values(self):
        sut = self._create_sut("4", "-3", MathOperation.MULTIPLY, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(-12.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('-12.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_multiply_negative_and_positive_values(self):
        sut = self._create_sut("-4", "3", MathOperation.MULTIPLY, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(-12.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('-12.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_divide_positive_values(self):
        sut = self._create_sut("3", "3", MathOperation.DIVIDE, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(1.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('1.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_divide_negative_values(self):
        sut = self._create_sut("-4", "-2", MathOperation.DIVIDE, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(2.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('2.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_divide_positive_and_negative_values(self):
        sut = self._create_sut("6", "-3", MathOperation.DIVIDE, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(-2.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('-2.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_divide_negative_and_positive_values(self):
        sut = self._create_sut("-6", "3", MathOperation.DIVIDE, self.CTX_SAVE_RESULT)
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(-2.0) % self.CTX_SAVE_RESULT)
        self.assertEqual('-2.0', self._context.get_info(self.CTX_SAVE_RESULT))

    def test_divide_by_zero(self):
        sut = self._create_sut("10", "0", MathOperation.DIVIDE, self.CTX_SAVE_RESULT)
        self._assert_run_throw_config_exception(sut, "Second value = 0 ! Division by 0 is not possible")

    def _create_sut(self, first, second, operator, save_result_as):
        self._sut = MathOperation(None, None, {"FIRST": first, "SECOND": second, "OPERATOR": operator, "SAVE_RESULT_AS": save_result_as},mock.Mock())
        return self._sut
