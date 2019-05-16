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
:since: 20/03/14
:author: fbongiax
"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Utilities.Checks.Compare import Compare


class CompareTestCase(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None

    def test_different_types_given(self):
        self._create_sut("10", "aaa", Compare.EQ)
        self._assert_run_throw_config_exception(self._sut, "FIRST and SECOND must be both either strings or numbers")

    def test_equal_integers(self):
        self._create_sut("10", "10", Compare.EQ)
        self._assert_run_succeeded(self._sut)

    def test_equal_integers_pass_if_false(self):
        self._create_sut("10", "10", Compare.EQ, pass_if=False)
        self._assert_run_throw_config_exception(self._sut, "Comparison is not satisfied")

    def test_not_equal_integers(self):
        self._create_sut("10", "4", Compare.NE)
        self._assert_run_succeeded(self._sut)

    def test_greater_than_integers(self):
        self._create_sut("10", "4", Compare.GT)
        self._assert_run_succeeded(self._sut)

    def test_less_than_integers(self):
        self._create_sut(2, 4, Compare.LT)
        self._assert_run_succeeded(self._sut)

    def test_great_or_equal_string(self):
        self._create_sut("bbbb", "aaaa", Compare.GE)
        self._assert_run_succeeded(self._sut)

    def test_less_or_equal_string(self):
        self._create_sut("first", "second", Compare.LE)
        self._assert_run_succeeded(self._sut)

    def test_not_equal_float(self):
        self._create_sut(5.4, 5.7, Compare.NE)
        self._assert_run_succeeded(self._sut)

    def test_in_str_pass_if_true_success(self):
        self._create_sut("bla", "blabla", Compare.IN, pass_if=True)
        self._assert_run_succeeded(self._sut)

    def test_in_str_pass_if_true_fails(self):
        self._create_sut("bbbb", "aaaa", Compare.IN, pass_if=True)
        self._assert_run_throw_config_exception(self._sut, "Comparison is not satisfied")

    def test_in_str_pass_if_false_success(self):
        self._create_sut("bli", "blabla", Compare.IN, pass_if=False)
        self._assert_run_succeeded(self._sut)

    def test_in_str_pass_if_false_fails(self):
        self._create_sut("blabla", "blabla", Compare.IN, pass_if=False)
        self._assert_run_throw_config_exception(self._sut, "Comparison is not satisfied")

    def test_in_int_success(self):
        self._create_sut("1", "123", Compare.IN, pass_if=True)
        self._assert_run_succeeded(self._sut)

    def test_in_int_fails(self):
        self._create_sut("4", "123", Compare.IN, pass_if=True)
        self._assert_run_throw_config_exception(self._sut, "Comparison is not satisfied")

    def test_in_float_fails(self):
        self._create_sut("1.5", "00031.4000", Compare.IN, pass_if=True)
        self._assert_run_throw_config_exception(self._sut, "Comparison is not satisfied")

    def test_in_float_succeeds(self):
        self._create_sut("1.5", "00031.5000", Compare.IN, pass_if=True)
        self._assert_run_succeeded(self._sut)

    def _create_sut(self, first, second, operator, pass_if=True):
        self._sut = Compare(None, None, {"FIRST": first, "SECOND": second, "OPERATOR": operator, "PASS_IF": pass_if},
                            mock.Mock())
        return self._sut
