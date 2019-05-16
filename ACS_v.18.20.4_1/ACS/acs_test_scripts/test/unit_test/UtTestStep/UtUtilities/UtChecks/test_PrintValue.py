# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the unit test for print value.
:since: 2014-10-23
:author: emarchan

"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Utilities.Checks.PrintValue import PrintValue

DEFAULT_VAR_NAME = "my_var"
class test_PrintValue(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None
        self._context = TestStepContext()

    def test_var_in_context(self):
        self._create_sut({"VALUE_TO_PRINT":"FROM_CTX:" + DEFAULT_VAR_NAME})
        self._context.set_info(DEFAULT_VAR_NAME, "MY_VALUE")
        self._assert_run_succeeded_with_msg(self._sut,
                                    "VERDICT: %s value is %s." % ("FROM_CTX:" + DEFAULT_VAR_NAME, "MY_VALUE"))

    def test_var_not_in_context(self):
        self._create_sut({"VALUE_TO_PRINT":"FROM_CTX:dummy"})
        self._assert_run_succeeded_with_msg(self._sut,
                            "VERDICT: %s value wasn't found." % ("FROM_CTX:dummy"))


    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        self._sut = PrintValue(None, None, test_step_pars, mock.MagicMock())
